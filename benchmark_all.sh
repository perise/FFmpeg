#!/bin/sh
# benchmark_all.sh — K1 RVV acceleration benchmark
# Usage: ./benchmark_all.sh [ffmpeg_binary]
# Compares RVV-accelerated vs C-fallback for each implemented direction.

FFMPEG="${1:-./ffmpeg_g}"
OUT=/mnt/sdcard/bench_results.txt
REPS=3          # runs per test; take best
W=1280; H=720   # test resolution

echo "=== K1 RVV Acceleration Benchmark ===" | tee $OUT
echo "Binary: $FFMPEG" | tee -a $OUT
echo "Resolution: ${W}x${H}" | tee -a $OUT
echo "Date: $(date)" | tee -a $OUT
echo "" | tee -a $OUT

# Helper: run ffmpeg and extract fps from stderr
bench_fps() {
    local label="$1"; shift
    local best=0
    local i=0
    while [ $i -lt $REPS ]; do
        fps=$("$FFMPEG" "$@" -benchmark 2>&1 | \
              grep -oE 'fps=[[:space:]]*[0-9]+(\.[0-9]+)?' | \
              grep -oE '[0-9]+(\.[0-9]+)?$' | tail -1)
        [ -z "$fps" ] && fps=0
        # keep the best (highest fps)
        awk -v b="$best" -v f="$fps" 'BEGIN{exit (f+0 > b+0) ? 0 : 1}' && best=$fps
        i=$((i+1))
    done
    printf "  %-40s %8s fps\n" "$label" "$best" | tee -a $OUT
    echo "$best"
}

bench_sec() {
    # returns elapsed seconds from -benchmark output
    local label="$1"; shift
    local best=9999
    local i=0
    while [ $i -lt $REPS ]; do
        sec=$("$FFMPEG" "$@" -benchmark 2>&1 | \
              grep "bench:" | grep -oE 'rtime=[0-9]+\.[0-9]+' | \
              grep -oE '[0-9]+\.[0-9]+')
        [ -z "$sec" ] && sec=9999
        awk -v b="$best" -v s="$sec" 'BEGIN{exit (s+0 < b+0) ? 0 : 1}' && best=$sec
        i=$((i+1))
    done
    printf "  %-40s %8s s\n" "$label" "$best" | tee -a $OUT
    echo "$best"
}

# ─── Direction 7: NLMeans filter ────────────────────────────────────────────
echo "--- Direction 7: NLMeans (nlmeans filter) ---" | tee -a $OUT
NL_ARGS="-f lavfi -i testsrc=size=${W}x${H}:rate=25:duration=4,format=yuv420p \
         -vf nlmeans -frames:v 100 -f null -"

printf "  %-40s\n" "RVV path:" | tee -a $OUT
rvv_nl=$(bench_fps  "  nlmeans RVV" $NL_ARGS)

printf "  %-40s\n" "C path (-cpuflags 0):" | tee -a $OUT
c_nl=$(bench_fps    "  nlmeans C  " -cpuflags 0 $NL_ARGS)

[ "$c_nl" != "0" ] && \
  awk -v r="$rvv_nl" -v c="$c_nl" \
    'BEGIN{printf "  Speedup: %.2fx\n", r/c}' | tee -a $OUT
echo "" | tee -a $OUT

# ─── Direction 9: Image copy (bulk plane copy) ───────────────────────────────
echo "--- Direction 9: Image copy (av_image_copy_plane) ---" | tee -a $OUT
# scale forces multiple image copies per frame
CP_ARGS="-f lavfi -i testsrc=size=${W}x${H}:rate=25:duration=4,format=yuv420p \
         -vf copy -frames:v 200 -f null -"

rvv_cp=$(bench_fps  "  imgcopy RVV" $CP_ARGS)
c_cp=$(bench_fps    "  imgcopy C  " -cpuflags 0 $CP_ARGS)
[ "$c_cp" != "0" ] && \
  awk -v r="$rvv_cp" -v c="$c_cp" \
    'BEGIN{printf "  Speedup: %.2fx\n", r/c}' | tee -a $OUT
echo "" | tee -a $OUT

# ─── Direction 6: Yadif deinterlace ─────────────────────────────────────────
echo "--- Direction 6: Yadif deinterlace ---" | tee -a $OUT
YD_ARGS="-f lavfi -i testsrc=size=${W}x${H}:rate=25:duration=4,format=yuv420p \
         -vf yadif -frames:v 100 -f null -"

rvv_yd=$(bench_fps  "  yadif RVV " $YD_ARGS)
c_yd=$(bench_fps    "  yadif C   " -cpuflags 0 $YD_ARGS)
[ "$c_yd" != "0" ] && \
  awk -v r="$rvv_yd" -v c="$c_yd" \
    'BEGIN{printf "  Speedup: %.2fx\n", r/c}' | tee -a $OUT
echo "" | tee -a $OUT

# ─── Direction 4: swscale horizontal (scale filter) ─────────────────────────
# echo "--- Direction 4: swscale horizontal scaler ---" | tee -a $OUT
SC_ARGS="-f lavfi -i testsrc=size=1920x1080:rate=25:duration=4,format=yuv420p \
         -vf scale=${W}:${H} -frames:v 100 -f null -"

rvv_sc=$(bench_fps  "  scale RVV " $SC_ARGS)
c_sc=$(bench_fps    "  scale C   " -cpuflags 0 $SC_ARGS)
[ "$c_sc" != "0" ] && \
  awk -v r="$rvv_sc" -v c="$c_sc" \
    'BEGIN{printf "  Speedup: %.2fx\n", r/c}' | tee -a $OUT
echo "" | tee -a $OUT

echo "=== Done. Results in $OUT ===" | tee -a $OUT
