/* Enable GNU extensions for cpu_set_t, pthread_setaffinity_np, CPU_ZERO/SET */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

/*
 * K1 MPP multi-stream cluster-aware scheduler
 * Copyright (C) 2024 perise
 *
 * SpacemiT K1 topology:
 *   Cluster A: cores 0-3  (share L2 cache)
 *   Cluster B: cores 4-7  (share L2 cache)
 *   Shared L3 + VPU/NPU
 *
 * Design:
 *   - Each transcode session (stream) is assigned to a cluster slot.
 *   - Software threads doing CPU work (RVV scale/filter) are pinned to
 *     that cluster so filter state stays hot in the cluster's L2.
 *   - MPP (VPU encode/decode) calls are serialised through a per-cluster
 *     job queue; this avoids thundering-herd on the single VPU hardware
 *     arbiter and reduces context-switch overhead.
 *   - A session acquires a slot at open time and releases it at close.
 *     Up to K1_MAX_STREAMS concurrent streams are supported; beyond that
 *     new streams fall back to an unaffinized path.
 *
 * Usage:
 *   k1_sched_init();                         // once at startup
 *   K1StreamSlot *slot = k1_sched_acquire(); // per stream open
 *   k1_sched_pin(slot);                      // call from worker thread
 *   k1_vpu_submit(slot, fn, arg);            // serialised MPP call
 *   k1_sched_release(slot);                  // per stream close
 *   k1_sched_destroy();                      // once at shutdown
 */

#ifndef K1_MPP_SCHEDULER_H
#define K1_MPP_SCHEDULER_H

#include <pthread.h>
#include <sched.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* Constants                                                           */
/* ------------------------------------------------------------------ */

#define K1_CLUSTER_A_FIRST   0
#define K1_CLUSTER_A_LAST    3
#define K1_CLUSTER_B_FIRST   4
#define K1_CLUSTER_B_LAST    7
#define K1_CORES_PER_CLUSTER 4
#define K1_NUM_CLUSTERS      2
#define K1_MAX_STREAMS       8   /* max concurrent streams (1 per core) */

/* ------------------------------------------------------------------ */
/* VPU job queue                                                        */
/* ------------------------------------------------------------------ */

typedef void (*K1VpuJobFn)(void *arg);

typedef struct K1VpuJob {
    K1VpuJobFn  fn;
    void       *arg;
    int         done;           /* set to 1 after fn() returns */
    pthread_cond_t  done_cond;
    pthread_mutex_t done_lock;
} K1VpuJob;

/* Per-cluster VPU serialisation queue (ring buffer, capacity = 16) */
#define K1_VPU_QUEUE_CAP 16

typedef struct K1VpuQueue {
    K1VpuJob   *ring[K1_VPU_QUEUE_CAP];
    int         head, tail;
    int         count;
    pthread_mutex_t lock;
    pthread_cond_t  not_empty;
    pthread_cond_t  not_full;
    pthread_t       worker;
    int             shutdown;
} K1VpuQueue;

/* ------------------------------------------------------------------ */
/* Stream slot                                                          */
/* ------------------------------------------------------------------ */

typedef struct K1StreamSlot {
    int  cluster;           /* 0 = A (cores 0-3), 1 = B (cores 4-7) */
    int  core_hint;         /* preferred core within cluster */
    int  slot_id;           /* index within K1_MAX_STREAMS */
    K1VpuQueue *queue;      /* back-pointer to cluster's VPU queue */
} K1StreamSlot;

/* ------------------------------------------------------------------ */
/* Scheduler state (file-scope singleton)                              */
/* ------------------------------------------------------------------ */

typedef struct K1Scheduler {
    K1VpuQueue      queues[K1_NUM_CLUSTERS];
    atomic_int      stream_count[K1_NUM_CLUSTERS]; /* active streams per cluster */
    pthread_mutex_t alloc_lock;
    int             initialized;
} K1Scheduler;

static K1Scheduler g_k1_sched;

/* ------------------------------------------------------------------ */
/* VPU worker thread                                                    */
/* ------------------------------------------------------------------ */

static void *k1_vpu_worker(void *arg)
{
    K1VpuQueue *q = (K1VpuQueue *)arg;

    /* Pin worker to cluster's cores (cluster derived from pointer arithmetic) */
    int cluster = (int)(q - g_k1_sched.queues);
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    int first = cluster ? K1_CLUSTER_B_FIRST : K1_CLUSTER_A_FIRST;
    int last  = cluster ? K1_CLUSTER_B_LAST  : K1_CLUSTER_A_LAST;
    for (int c = first; c <= last; c++)
        CPU_SET(c, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);

    while (1) {
        pthread_mutex_lock(&q->lock);
        while (q->count == 0 && !q->shutdown)
            pthread_cond_wait(&q->not_empty, &q->lock);
        if (q->shutdown && q->count == 0) {
            pthread_mutex_unlock(&q->lock);
            break;
        }
        K1VpuJob *job = q->ring[q->head];
        q->head = (q->head + 1) % K1_VPU_QUEUE_CAP;
        q->count--;
        pthread_cond_signal(&q->not_full);
        pthread_mutex_unlock(&q->lock);

        /* Execute VPU job outside the queue lock */
        job->fn(job->arg);

        pthread_mutex_lock(&job->done_lock);
        job->done = 1;
        pthread_cond_signal(&job->done_cond);
        pthread_mutex_unlock(&job->done_lock);
    }
    return NULL;
}

/* ------------------------------------------------------------------ */
/* Public API                                                           */
/* ------------------------------------------------------------------ */

/**
 * Initialise the scheduler singleton.  Call once before any streams open.
 * Safe to call multiple times (idempotent via initialized flag).
 */
static inline int k1_sched_init(void)
{
    if (g_k1_sched.initialized)
        return 0;

    pthread_mutex_init(&g_k1_sched.alloc_lock, NULL);

    for (int cl = 0; cl < K1_NUM_CLUSTERS; cl++) {
        K1VpuQueue *q = &g_k1_sched.queues[cl];
        memset(q, 0, sizeof(*q));
        pthread_mutex_init(&q->lock, NULL);
        pthread_cond_init(&q->not_empty, NULL);
        pthread_cond_init(&q->not_full, NULL);
        atomic_store(&g_k1_sched.stream_count[cl], 0);

        if (pthread_create(&q->worker, NULL, k1_vpu_worker, q) != 0)
            return -1;
    }
    g_k1_sched.initialized = 1;
    return 0;
}

/**
 * Acquire a stream slot.  Assigns the stream to the less-loaded cluster.
 * Returns NULL if K1_MAX_STREAMS already active (caller falls back to
 * unaffinized MPP path).
 */
static inline K1StreamSlot *k1_sched_acquire(void)
{
    if (!g_k1_sched.initialized)
        k1_sched_init();

    pthread_mutex_lock(&g_k1_sched.alloc_lock);

    int total = atomic_load(&g_k1_sched.stream_count[0]) +
                atomic_load(&g_k1_sched.stream_count[1]);
    if (total >= K1_MAX_STREAMS) {
        pthread_mutex_unlock(&g_k1_sched.alloc_lock);
        return NULL;
    }

    /* Pick least-loaded cluster */
    int ca = atomic_load(&g_k1_sched.stream_count[0]);
    int cb = atomic_load(&g_k1_sched.stream_count[1]);
    int cluster = (ca <= cb) ? 0 : 1;

    int idx = atomic_fetch_add(&g_k1_sched.stream_count[cluster], 1);
    pthread_mutex_unlock(&g_k1_sched.alloc_lock);

    K1StreamSlot *slot = (K1StreamSlot *)malloc(sizeof(K1StreamSlot));
    if (!slot) {
        atomic_fetch_sub(&g_k1_sched.stream_count[cluster], 1);
        return NULL;
    }
    slot->cluster   = cluster;
    slot->core_hint = (cluster ? K1_CLUSTER_B_FIRST : K1_CLUSTER_A_FIRST) + (idx % K1_CORES_PER_CLUSTER);
    slot->slot_id   = total;
    slot->queue     = &g_k1_sched.queues[cluster];
    return slot;
}

/**
 * Pin the calling thread to the slot's cluster.
 * Call this from the FFmpeg worker thread right after acquiring the slot
 * (e.g., inside AVCodecContext's thread_init callback or at filter init).
 *
 * Pins to the full cluster (all 4 cores) so the OS can still balance
 * within-cluster; use K1_SCHED_PIN_TIGHT to pin to one specific core.
 */
static inline void k1_sched_pin(const K1StreamSlot *slot)
{
    if (!slot) return;
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    int first = slot->cluster ? K1_CLUSTER_B_FIRST : K1_CLUSTER_A_FIRST;
    int last  = slot->cluster ? K1_CLUSTER_B_LAST  : K1_CLUSTER_A_LAST;
    for (int c = first; c <= last; c++)
        CPU_SET(c, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
}

/**
 * Pin the calling thread to a single core within the slot's cluster.
 * Use for highly cache-sensitive work (e.g., RVV filter inner loops).
 */
static inline void k1_sched_pin_tight(const K1StreamSlot *slot)
{
    if (!slot) return;
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(slot->core_hint, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
}

/**
 * Submit a synchronous VPU job on the slot's cluster queue.
 * Blocks until fn(arg) has completed on the VPU worker thread.
 * This serialises MPP encode/decode calls per-cluster, preventing
 * concurrent access to the VPU hardware arbiter.
 */
static inline int k1_vpu_submit(K1StreamSlot *slot, K1VpuJobFn fn, void *arg)
{
    if (!slot) {
        /* No slot: call directly (fallback for >8 streams) */
        fn(arg);
        return 0;
    }

    K1VpuJob job;
    job.fn  = fn;
    job.arg = arg;
    job.done = 0;
    pthread_mutex_init(&job.done_lock, NULL);
    pthread_cond_init(&job.done_cond, NULL);

    K1VpuQueue *q = slot->queue;
    pthread_mutex_lock(&q->lock);
    while (q->count >= K1_VPU_QUEUE_CAP)
        pthread_cond_wait(&q->not_full, &q->lock);
    q->ring[q->tail] = &job;
    q->tail = (q->tail + 1) % K1_VPU_QUEUE_CAP;
    q->count++;
    pthread_cond_signal(&q->not_empty);
    pthread_mutex_unlock(&q->lock);

    /* Wait for completion */
    pthread_mutex_lock(&job.done_lock);
    while (!job.done)
        pthread_cond_wait(&job.done_cond, &job.done_lock);
    pthread_mutex_unlock(&job.done_lock);

    pthread_mutex_destroy(&job.done_lock);
    pthread_cond_destroy(&job.done_cond);
    return 0;
}

/**
 * Release a stream slot.  Must be called when the stream closes.
 */
static inline void k1_sched_release(K1StreamSlot *slot)
{
    if (!slot) return;
    atomic_fetch_sub(&g_k1_sched.stream_count[slot->cluster], 1);
    free(slot);
}

/**
 * Shut down the scheduler.  Waits for queued VPU jobs to drain.
 * Call after all streams have released their slots.
 */
static inline void k1_sched_destroy(void)
{
    if (!g_k1_sched.initialized)
        return;
    for (int cl = 0; cl < K1_NUM_CLUSTERS; cl++) {
        K1VpuQueue *q = &g_k1_sched.queues[cl];
        pthread_mutex_lock(&q->lock);
        q->shutdown = 1;
        pthread_cond_signal(&q->not_empty);
        pthread_mutex_unlock(&q->lock);
        pthread_join(q->worker, NULL);
        pthread_mutex_destroy(&q->lock);
        pthread_cond_destroy(&q->not_empty);
        pthread_cond_destroy(&q->not_full);
    }
    pthread_mutex_destroy(&g_k1_sched.alloc_lock);
    g_k1_sched.initialized = 0;
}

/**
 * Convenience: query which cluster the calling thread is on.
 * Returns -1 if not pinned by the scheduler.
 */
static inline int k1_sched_current_cluster(void)
{
    cpu_set_t cpuset;
    if (pthread_getaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) != 0)
        return -1;
    /* Check cluster A */
    int in_a = 0, in_b = 0;
    for (int c = K1_CLUSTER_A_FIRST; c <= K1_CLUSTER_A_LAST; c++)
        if (CPU_ISSET(c, &cpuset)) in_a++;
    for (int c = K1_CLUSTER_B_FIRST; c <= K1_CLUSTER_B_LAST; c++)
        if (CPU_ISSET(c, &cpuset)) in_b++;
    if (in_a > 0 && in_b == 0) return 0;
    if (in_b > 0 && in_a == 0) return 1;
    return -1; /* spans clusters or unset */
}

#ifdef __cplusplus
}
#endif

#endif /* K1_MPP_SCHEDULER_H */
