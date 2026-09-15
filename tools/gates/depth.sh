#!/usr/bin/env bash
#
# P4 gate: metric depth on the GPU, at the rate the GPU can sustain, dated to when
# the light arrived.
#
# Four claims, and the fourth is a control.
#
#  1. **The provider is CUDA**, read from depth_node's own startup line. A CPU
#     fallback is a failed test however good the cloud looks — that is P4's
#     wording, and it is the whole reason this phase was written as toolchain
#     work first.
#  2. **Per-frame cost within budget**, from the node's own log line. A per-frame
#     cost *is* the interval between entering and leaving the work, on one clock,
#     and the only clock that sees both ends is the node's. The budget is on the
#     whole per-frame cost and not on inference alone: preprocessing, the
#     reciprocal, the resize back to 1280x720 and two publishes all go on top of
#     the 51.20 ms `tools/gates/gpu-stack.sh` measured for inference by itself.
#  3. **`/depth/rgb` is byte-identical to the frame its depth map was inferred
#     on**, and every `/depth` carries a stamp the input actually had. This is
#     the assertion that catches a whole class of silent wrongness: a depth map
#     paired with a *different* frame produces a coloured cloud that looks
#     entirely plausible and is registered to the wrong instant. Nothing
#     downstream could ever tell.
#  4. **The CPU path fails the same budget** — the control, run second. A budget
#     nobody has watched fail is not an assertion, and this one is worth
#     particular care because the failure it guards against is *silent*: ONNX
#     Runtime will run this model on the CPU at four times the cost and say
#     nothing about it.
#
# **The container is what is measured, and that is not a detail — it is where
# this phase's real bug lived.** `tools/gates/gpu-stack.sh` proves the GPU stack
# works for a program we link ourselves, and that proof does **not** carry to a
# component loaded into somebody else's container: the executable's `DT_RPATH` is
# the only one glibc consults for a dlopened provider's dependencies, and
# `component_container_isolated` has none. Measured 2026-09-15, one container
# apart on identical libraries: `gpu_probe` at 51 ms on CUDA, `depth_node` at
# 517 ms on the CPU, with the pipeline around it working perfectly and the gate
# that was supposed to cover this saying PASS. See `preload_cuda_provider()` in
# src/depth_engine_ort.cpp. Ask what the gate does **not** touch.
#
# It replays bags/desk1 rather than using the camera, like every phase from P3
# on, so the numbers compare like for like. The Pi is not involved at all.

source "$(dirname "${BASH_SOURCE[0]}")/../just-lib.sh" --overlay
echo "== gate-depth =="

BAG_NAME=${1:-desk1}

# 80 ms, from P4. The predecessor measured 72-79 ms for inference alone on this
# card through Python; ours measures 51.20 ms for inference alone in C++
# (gates/gpu-stack.sh) and this budget covers the whole frame around it.
MAX_COST_MS=80.0
# Depth is the pipeline's clock and everything downstream inherits this rate, so
# it is asserted rather than merely printed. ~59 Hz in and ~55 ms a frame gives
# ~18 Hz; the floor is well under that because a floor is not a target.
MIN_RATE_HZ=10
# The control has to be *worse than the budget*, not merely worse than CUDA.
# "Slower than the GPU" would be satisfied by 81 ms and prove nothing about a
# threshold set at 80.
CONTROL_SECONDS=25
# How much of the clip has to reach the container. Not 100%: the first frames go
# past while the probe is still connecting.
MIN_CLIP_COVERAGE_PCT=85
# Near and far have to differ. A depth map that is one distance everywhere is
# exactly what a reciprocal applied on the wrong side of the clamp produces, and
# it renders as a convincing flat wall.
MIN_DEPTH_SPREAD_M=0.5

# Before arm_cleanup, always — see assert_no_session in tools/just-lib.sh: the
# cleanup handler kills this workspace's processes, so a refusal after the trap
# is armed would tear down the session it is refusing to disturb.
assert_no_session "bash tools/gates/depth.sh"

arm_cleanup kill_local

fail=0
note() { echo "FAIL: $*"; fail=1; }

# --- The model, before anything is started ----------------------------------
#
# Checked first and by name, because the alternative is a container that comes up,
# fails to load one component, and carries on — which reads as a pipeline fault
# rather than a missing 99 MB file that a script downloads.
MODEL="$PIMESH_WS/models/depth_anything_v2_small.onnx"
[[ -r $MODEL ]] || {
    echo "FAIL: no model at ${MODEL}"
    echo
    echo "models/ is git-ignored, so a fresh clone has none. Fetch it with:"
    echo "  bash tools/fetch-model.sh"
    exit 1
}

BAG=
for cand in "$BAG_NAME" "$PIMESH_WS/bags/$BAG_NAME"; do
    [[ -r $cand/metadata.yaml ]] && { BAG=$cand; break; }
done
[[ -n $BAG ]] || {
    echo "FAIL: no bag at '${BAG_NAME}' (looked for metadata.yaml there and under bags/)"
    echo
    echo "The reference clip is recorded once, with:"
    echo "  bash tools/record-clip.sh desk1 60"
    echo
    echo "bags/ is git-ignored, so a fresh clone has none — this gate cannot be"
    echo "run without one and does not pretend otherwise."
    exit 1
}
echo "clip : ${BAG}"
echo "model: ${MODEL}"

work=$(mktemp -d)

read -r CLIP_SECONDS CLIP_FRAMES < <(/usr/bin/python3 - "$BAG/metadata.yaml" <<'META'
import sys
import yaml

with open(sys.argv[1]) as handle:
    info = yaml.safe_load(handle)['rosbag2_bagfile_information']
images = 0
for entry in info['topics_with_message_count']:
    if entry['topic_metadata']['name'].endswith('image_raw/compressed'):
        images = entry['message_count']
print(f"{info['duration']['nanoseconds'] / 1e9:.1f} {images}")
META
)
[[ -n ${CLIP_FRAMES:-} && $CLIP_FRAMES -gt 100 ]] || {
    echo "FAIL: ${BAG} holds ${CLIP_FRAMES:-0} images on /image_raw/compressed"
    exit 1
}
MEASURE_S=$(awk -v s="$CLIP_SECONDS" 'BEGIN { printf "%d", s + 1 }')
echo "       ${CLIP_FRAMES} frames over ${CLIP_SECONDS}s"

# --- One run of the real pipeline against the clip ---------------------------
#
# The real launch file, so what is measured is the configuration that runs: one
# container, intra-process comms on, decode_node and keypoint_node beside
# depth_node, and the three static edges. probe:=depth_probe loads the instrument
# into the same process — out of process it would be subscribing to ~255 MB/s of
# images and would be the dominant load on the thing it is measuring.
#
# `probe` names the instrument rather than switching one on, because there are two
# of them now and loading the IPC probe beside this one would put a third consumer
# on /image_raw during the measurement.
run_pipeline() {        # $1 = log path, $2 = window seconds, $3 = true|false (CUDA)
    local log=$1 window=$2 cuda=$3

    timeout -s INT $(( window + 45 )) ros2 launch pimesh_bringup pimesh.launch.py \
        probe:=depth_probe use_cuda:="$cuda" probe_duration_s:="${window}.0" \
        >"$log" 2>&1 &

    # Wait on the launcher's own log line, not on `ros2 node list`: the daemon
    # caches discovery state and a stale cache has already made one gate in this
    # repo fail while the log beside it said the node was loaded. The container's
    # load confirmation is first-hand and needs nothing else running.
    #
    # Generous, because this one has a model to load and a session to warm: 99 MB
    # off disk, a CUDA context, cuBLAS handles and kernel autotuning.
    local ready=0
    for _ in $(seq 120); do
        if grep -q "depth_node up:" "$log" 2>/dev/null; then ready=1; break; fi
        sleep 0.5
    done
    (( ready == 1 )) || {
        echo "FAIL: depth_node never came up within 60 s"
        tail -40 "$log"
        return 1
    }

    # Once, not --loop. A looping bag replays header stamps ~60 s into the past at
    # every wrap, which would make claim 3 a measurement of the wrap rather than of
    # the pairing. Both halves of the terminal handling are here for the reason
    # tools/replay.sh documents: a backgrounded `ros2 bag play` that can read its
    # controlling TTY is sent SIGTTIN and stops, silently, publishing nothing.
    timeout -s INT $(( window + 20 )) ros2 bag play "$BAG" \
        --disable-keyboard-controls </dev/null >"$log.play" 2>&1 &

    sleep $(( window + 4 ))
    kill_local
    sleep 1
}

# The probe reports on stdout, which for a composed component is the container's
# stdout and therefore the launch log — with `[component_container_isolated-N] `
# prefixed to every line by the launcher. Stripping that first is the whole reason
# this is a function: the obvious `awk -F=` over the raw log silently matches
# nothing, because the first field is the prefix rather than the key, and a gate
# that reads every value as empty reports "measured 0" over a run that measured
# 1047. Found exactly that way, 2026-09-15.
probe_value() {         # $1 = log, $2 = key
    sed 's/^\[[^]]*\] //' "$1" |
        awk -F= -v key="$2" '$1 == "probe " key {print $2}' | tail -1
}

# --- The node's own stats line, aggregated over every window it logged --------
#
# Not `tail -1`: the last window of a run covers the seconds *after* the clip
# ended, when the node is idle and its mean is 0.00 over no frames. Windows with
# no frames are dropped and the rest are averaged — they are all the same length,
# so a mean of means is the mean.
#
# The p95 reported is the **worst window's** p95, which is an upper bound on the
# run's p95 rather than the run's p95 itself. Said plainly because it is the
# conservative direction and the honest label: a p95 over ~90 samples cannot be
# pooled across windows from the summaries alone.
cost_summary() {        # $1 = log; prints "windows mean_ms worst_p95_ms max_ms"
    grep -h 'depth_node' "$1" | grep -o 'stats rate=.*' |
        awk '{
            for (i = 1; i <= NF; ++i) {
                split($i, kv, "=")
                v[kv[1]] = kv[2]
            }
            if (v["rate"] + 0 <= 0) { next }
            n++
            sum += v["cost_mean"]
            if (v["cost_p95"] + 0 > p95) { p95 = v["cost_p95"] }
            if (v["max"] + 0 > mx) { mx = v["max"] }
        }
        END {
            if (n == 0) { print "0 0 0 0"; exit }
            printf "%d %.2f %.2f %.2f\n", n, sum / n, p95, mx
        }'
}

provider_of() {         # $1 = log
    sed -n 's/.*inference provider: \([A-Za-z]*\).*/\1/p' "$1" | tail -1
}

# =============================================================================
# Run 1 — the measurement
# =============================================================================
echo
echo "-- run 1: the pipeline on the GPU, over the whole clip --"
run_pipeline "$work/cuda.log" "$MEASURE_S" true || { echo "FAIL gate-depth"; exit 1; }

provider=$(provider_of "$work/cuda.log")
read -r windows cost_mean cost_p95 cost_max < <(cost_summary "$work/cuda.log")

source_frames=$(probe_value "$work/cuda.log" source_frames)
rgb_frames=$(probe_value "$work/cuda.log" rgb_frames)
depth_frames=$(probe_value "$work/cuda.log" depth_frames)
measured=$(probe_value "$work/cuda.log" measured)
rate=$(probe_value "$work/cuda.log" rate_hz)
interval_p95=$(probe_value "$work/cuda.log" interval_p95_ms)
rgb_identical=$(probe_value "$work/cuda.log" rgb_identical)
rgb_different=$(probe_value "$work/cuda.log" rgb_different)
rgb_unmatched=$(probe_value "$work/cuda.log" rgb_unmatched)
stamp_matched=$(probe_value "$work/cuda.log" stamp_matched)
stamp_unmatched=$(probe_value "$work/cuda.log" stamp_unmatched)
wrong_encoding=$(probe_value "$work/cuda.log" wrong_encoding)
wrong_frame=$(probe_value "$work/cuda.log" wrong_frame)
sampled=$(probe_value "$work/cuda.log" sampled)
non_finite=$(probe_value "$work/cuda.log" non_finite)
out_of_range=$(probe_value "$work/cuda.log" out_of_range)
depth_mean_m=$(probe_value "$work/cuda.log" depth_mean_m)
depth_min_m=$(probe_value "$work/cuda.log" depth_min_m)
depth_max_m=$(probe_value "$work/cuda.log" depth_max_m)

if [[ -z ${measured:-} || ${measured:-0} -lt 50 ]]; then
    note "the probe measured ${measured:-0} depth maps — nothing below can be read"
    grep -iE 'error|refused|cannot' "$work/cuda.log" | head -10
    tail -20 "$work/cuda.log"
    echo "FAIL gate-depth"
    exit 1
fi

# --- Claim 1: the provider ---------------------------------------------------
[[ ${provider:-} == CUDAExecutionProvider ]] ||
    note "depth_node reports ${provider:-no provider at all} — a CPU fallback is a failed test"

# --- Claim 2: per-frame cost, on the node's own clock ------------------------
if (( windows == 0 )); then
    note "depth_node logged no stats window with frames in it — cost cannot be read"
else
    in_range "$cost_mean" 0 "$MAX_COST_MS" ||
        note "mean per-frame cost ${cost_mean} ms over ${windows} windows, budget is ${MAX_COST_MS} ms"
fi
in_range "$rate" "$MIN_RATE_HZ" 1000 ||
    note "sustained ${rate} Hz on /depth, floor is ${MIN_RATE_HZ} Hz"

# --- Claim 3: the colour twin and the stamps ---------------------------------
#
# Three separate counters and they mean three different things. `different` is the
# failure. `unmatched` is "we could not check", which is not a pass — a run that
# checked nothing would otherwise report zero mismatches and look perfect.
(( ${rgb_different:-1} == 0 )) ||
    note "${rgb_different} of ${rgb_frames:-?} /depth/rgb frames differed from the \
/image_raw frame with the same stamp — depth is paired with the wrong picture"
(( ${rgb_identical:-0} > 0 )) ||
    note "not one /depth/rgb frame was confirmed identical to its source — claim 3 was not measured"
(( ${rgb_unmatched:-1} == 0 )) ||
    note "${rgb_unmatched} /depth/rgb frames carried a stamp no /image_raw frame had"
(( ${stamp_unmatched:-1} == 0 )) ||
    note "${stamp_unmatched} /depth maps carried a stamp no /image_raw frame had — \
the stamp is not the input frame's"
(( ${wrong_encoding:-1} == 0 )) || note "${wrong_encoding} /depth maps were not 32FC1"
(( ${wrong_frame:-1} == 0 )) ||
    note "${wrong_frame} messages were not stamped camera_optical_frame"

# --- Are the distances usable numbers? ---------------------------------------
(( ${non_finite:-1} == 0 )) ||
    note "${non_finite} of ${sampled} sampled distances were NaN or infinite — \
one of those integrated into a TSDF poisons voxels that were fine"
(( ${out_of_range:-1} == 0 )) ||
    note "${out_of_range} of ${sampled} sampled distances were outside (0, max_range]"
spread=$(awk -v a="${depth_max_m:-0}" -v b="${depth_min_m:-0}" 'BEGIN { printf "%.3f", a - b }')
in_range "$spread" "$MIN_DEPTH_SPREAD_M" 1000 ||
    note "near and far differ by only ${spread} m — a depth map at one distance \
everywhere is what a reciprocal on the wrong side of the clamp produces"

coverage_pct=$(awk -v seen="${source_frames:-0}" -v total="$CLIP_FRAMES" \
    'BEGIN { printf "%.1f", 100 * seen / total }')
in_range "$coverage_pct" "$MIN_CLIP_COVERAGE_PCT" 110 ||
    note "only ${coverage_pct}% of the clip's ${CLIP_FRAMES} frames reached the container \
(floor ${MIN_CLIP_COVERAGE_PCT}%) — the numbers above cover less of the room than they claim"

# =============================================================================
# Run 2 — the control
# =============================================================================
#
# The same binary, the same clip, the same budget, one parameter apart. This is
# the run that gives the 80 ms threshold its meaning: without it the assertion
# above is a number that has never been seen to exclude anything, and the failure
# it is guarding against — ONNX Runtime quietly choosing the CPU — produces no
# error message anywhere.
echo
echo "-- run 2: the control, use_cuda:=false, same budget --"
run_pipeline "$work/cpu.log" "$CONTROL_SECONDS" false || { echo "FAIL gate-depth"; exit 1; }

control_provider=$(provider_of "$work/cpu.log")
read -r c_windows c_cost_mean c_cost_p95 c_cost_max < <(cost_summary "$work/cpu.log")
control_rate=$(probe_value "$work/cpu.log" rate_hz)

[[ ${control_provider:-} == CPUExecutionProvider ]] ||
    note "the control run reports ${control_provider:-no provider} — use_cuda:=false did not reach the node, \
so it is not a control"
if (( c_windows == 0 )); then
    note "the control run logged no stats window with frames in it"
else
    # The assertion is that it is *outside* the budget. A control that passed would
    # mean the budget does not discriminate between this GPU and this CPU, and the
    # number above would stop being evidence of anything.
    in_range "$c_cost_mean" "$MAX_COST_MS" 100000 ||
        note "the CPU control cost ${c_cost_mean} ms/frame, which is inside the \
${MAX_COST_MS} ms budget — a budget that the thing it excludes can meet is not an assertion"
fi

speedup=$(awk -v cpu="${c_cost_mean:-0}" -v gpu="${cost_mean:-1}" \
    'BEGIN { printf "%.2f", (gpu > 0) ? cpu / gpu : 0 }')

echo
echo "clip                : $(basename "$BAG")  (${CLIP_SECONDS}s, ${CLIP_FRAMES} frames)"
echo "frames into decode  : ${source_frames} of ${CLIP_FRAMES} = ${coverage_pct}%  (assert >= ${MIN_CLIP_COVERAGE_PCT}%)"
echo
echo "provider            : ${provider}  (assert CUDAExecutionProvider)"
echo "per-frame cost      : ${cost_mean} ms mean over ${windows} windows  (assert <= ${MAX_COST_MS}, node's own clock)"
echo "  ... p95           : ${cost_p95} ms  (the worst window's p95: an upper bound on the run's)"
echo "  ... worst frame   : ${cost_max} ms"
echo "sustained rate      : ${rate} Hz on /depth  (assert >= ${MIN_RATE_HZ}; the pipeline's clock)"
echo "  ... p95 interval  : ${interval_p95} ms  (printed: a mean hides a stall)"
echo "depth maps measured : ${measured} of ${depth_frames} seen  (warm_up_frames from config/pimesh.yaml)"
echo
echo "/depth/rgb identical: ${rgb_identical}  (assert > 0, and 0 different, 0 unmatched)"
echo "  ... different     : ${rgb_different}"
echo "  ... unmatched     : ${rgb_unmatched}  ('could not check' is not a pass)"
echo "stamps from input   : ${stamp_matched} matched, ${stamp_unmatched} not  (assert 0 not)"
echo "encoding / frame_id : ${wrong_encoding} wrong encoding, ${wrong_frame} wrong frame  (assert 0)"
echo
echo "distances sampled   : ${sampled}  (${non_finite} non-finite, ${out_of_range} out of range; assert 0 both)"
echo "  ... mean          : ${depth_mean_m} m"
echo "  ... near / far    : ${depth_min_m} m / ${depth_max_m} m, spread ${spread} m  (assert >= ${MIN_DEPTH_SPREAD_M})"
echo "  ... scale         : arbitrary. Monocular depth is scale-ambiguous and P5 is"
echo "                      what pins it. These metres are plausibly shaped and the"
echo "                      wrong size, and this gate deliberately does not assert on them."
echo
echo "control (use_cuda:=false)"
echo "  ... provider      : ${control_provider}  (assert CPUExecutionProvider)"
echo "  ... per-frame cost: ${c_cost_mean} ms over ${c_windows} windows  (assert > ${MAX_COST_MS}: the budget must exclude it)"
echo "  ... p95           : ${c_cost_p95} ms, worst frame ${c_cost_max} ms"
echo "  ... rate          : ${control_rate} Hz"
echo "  ... speed-up      : ${speedup}x"

(( fail == 0 )) || { echo "FAIL gate-depth"; exit 1; }
echo "PASS gate-depth"
