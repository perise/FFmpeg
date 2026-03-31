#!/bin/sh
# quickbench.sh — fast K1 RVV benchmark (320x240, 30 frames each)
FFMPEG="/mnt/sdcard/k1-ffmpeg-build/jellyfin-ffmpeg-k1/ffmpeg_g"
cd /mnt/sdcard/k1-ffmpeg-build/jellyfin-ffmpeg-k1
OUT=/mnt/sdcard/quickbench.txt
W=320; H=240; DUR=2; FPS=15; FRAMES=30

echo "=== K1 RVV Quick Benchmark ===" | tee $OUT
echo "Date: $(date)" | tee -a $OUT
echo "Res: ${W}x${H}  frames=${FRAMES}" | tee -a $OUT
echo "" | tee -a $OUT

run_one() {
    local tag="$1"; shift
    # Extract speed multiplier from ffmpeg output (e.g. speed=4.5x)
    local t0=$(date +%s%3N)
    "$FFMPEG" "$@" -f null - 2>/tmp/qb_err.txt
    local rc=$?
    local t1=$(date +%s%3N)
    local ms=$((t1 - t0))
    if [ $rc -eq 0 ]; then
        local spd=$(grep -oE 'speed=[[:space:]]*[0-9.]+' /tmp/qb_err.txt | tail -1 | grep -oE '[0-9.]+')
        printf "  %-44s %6d ms  speed=%sx\n" "$tag" "$ms" "${spd:-?}" | tee -a $OUT
    else
        printf "  %-44s  FAILED (rc=%d)\n" "$tag" "$rc" | tee -a $OUT
    fi
}

SRC="-f lavfi -i testsrc=size=${W}x${H}:rate=${FPS}:duration=${DUR},format=yuv420p"

# NLMeans
echo "--- Direction 7: NLMeans ---" | tee -a $OUT
run_one "nlmeans RVV"       $SRC -vf nlmeans   -frames:v $FRAMES
run_one "nlmeans C (cpuflags=0)" $SRC -cpuflags 0 -vf nlmeans -frames:v $FRAMES
echo "" | tee -a $OUT

# Image copy (passthrough exercises av_image_copy)
echo "--- Direction 9: Image copy ---" | tee -a $OUT
run_one "imgcopy RVV"       $SRC -vf copy      -frames:v $FRAMES
run_one "imgcopy C (cpuflags=0)" $SRC -cpuflags 0 -vf copy -frames:v $FRAMES
echo "" | tee -a $OUT

# Yadif deinterlace
echo "--- Direction 6: Yadif ---" | tee -a $OUT
run_one "yadif RVV"         $SRC -vf yadif     -frames:v $FRAMES
run_one "yadif C (cpuflags=0)"   $SRC -cpuflags 0 -vf yadif -frames:v $FRAMES
echo "" | tee -a $OUT

echo "=== Done ===" | tee -a $OUT
cat $OUT
