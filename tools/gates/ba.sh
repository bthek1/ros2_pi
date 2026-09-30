#!/usr/bin/env bash
#
# P15 gate (#11): local bundle adjustment on the backend thread.
#
# Replays TUM fr1/desk in three arms, alternating, several times each:
#
#   ba     local_map:=true  local_ba:=true    what is being measured
#   noba   local_map:=true  local_ba:=false   the plan's control: one parameter apart
#   p7     local_map:=false local_ba:=false   P7's tracker, which P14 did not beat
#
# The plan names only the first two. **The third is here because P14 happened**:
# the local map alone measured *worse* than P7's tracker (gates/map.sh, 2026-09-30),
# so "BA beats no-BA" can be true of a system that is still worse than the one the
# project had before milestone G started. It is printed, and whether BA also beats
# P7 is said in the output, but it is not asserted — it is not the claim P15 makes.
#
# --- The plan's two false greens, and what answers each ------------------------
#
#  1. **A BA that converges to a lower cost and a worse trajectory.** Chi-squared is
#     internal. The assertion is on the ATE, from evo, against motion capture.
#  2. **A BA that never runs** — an empty window, a solve that stops at iteration 0,
#     a write-back that does nothing — produces a run identical to the control and a
#     gate that passes because the control passes. Asserted: solves counted, mean
#     iterations > 0, mean free keyframes > 1, and the ATE *differs* from the
#     control's at all.
#
# And the one the plan adds for the system rather than the trajectory: **every
# upstream stage keeps its rate**. A backend that improves the pose by starving
# depth_node has not improved the pipeline — mesh_node's extraction did exactly that
# at equal priority. The bound is the no-BA runs' own spread plus slack, the form
# gates/dashboard.sh uses, because two identical runs already differ and a flat
# percentage would be a gate that fails on the weather. Only a slowdown fails.
#
# Nothing here touches the Pi.

source "$(dirname "${BASH_SOURCE[0]}")/../lib/just-lib.sh" --overlay
echo "== gate-ba =="

RUNS=${PIMESH_BA_RUNS:-3}
# The BA arm's depth model: 0 is one independent prior per reading, positive is a
# scale per keyframe that its readings share, with a log prior this wide. Only the
# ba arm gets it — the controls are what they were.
SCALE_SIGMA=${PIMESH_BA_SCALE_SIGMA:-0.0}
# The plan: "window size > 1". Free keyframes per solve, mean over the run.
MIN_WINDOW_FREE=1.5
# Slack over the measured noise floor, as a fraction.
RATE_SLACK=0.02
# A stage's windows below this rate are the idle tail or the start, not the run.
RUNNING_HZ=5
# Depth's first window after the replay starts is a burst above the source's own
# rate; windows above this are start-up, not steady state.
MAX_PLAUSIBLE_HZ=31

assert_no_session "bash tools/gates/ba.sh"
arm_cleanup kill_local

fail=0
note() { echo "FAIL: $*"; fail=1; }

EVO_APE=${EVO_APE:-$HOME/.local/bin/evo_ape}
MODEL="$PIMESH_WS/models/depth_anything_v2_small.onnx"
[[ -r $MODEL ]] || { echo "FAIL: no model at ${MODEL} — bash tools/fetch-model.sh"; exit 1; }
SEQ=$(bash "$PIMESH_WS/tools/fetch-dataset.sh" --print-path)
GROUND_TRUTH="$SEQ/groundtruth.txt"
[[ -r $GROUND_TRUTH ]] || { echo "FAIL: no dataset at ${SEQ} — bash tools/fetch-dataset.sh"; exit 1; }
[[ -x $EVO_APE ]] || { echo "FAIL: no evo at ${EVO_APE} — uv tool install evo"; exit 1; }

work=$(mktemp -d)
SEQ_SECONDS=$(awk '!/^#/ && NF { if (!s) s = $1; e = $1 } END { printf "%d", e - s }' "$SEQ/rgb.txt")
MEASURE_S=$(( SEQ_SECONDS + 8 ))

run_dataset() {          # $1 = log, $2 = local_map, $3 = local_ba, $4 = trajectory, $5 = scale sigma
    local log=$1 local_map=$2 local_ba=$3 traj=$4 scale_sigma=${5:-0.0}
    timeout -s INT $(( MEASURE_S + 45 )) ros2 launch pimesh_bringup pimesh.launch.py \
        source:=dataset_node dataset_dir:="$SEQ" odom_regime:=sixdof \
        local_map:="$local_map" local_ba:="$local_ba" ba_depth_scale_sigma:="$scale_sigma" \
        probe:=odom_probe \
        probe_duration_s:="$(( MEASURE_S + 4 ))" trajectory_path:="$traj" \
        >"$log" 2>&1 &
    local ready=0
    for _ in $(seq 120); do
        if grep -q "dataset .*replaying at" "$log" 2>/dev/null; then ready=1; break; fi
        sleep 0.5
    done
    (( ready == 1 )) || { echo "FAIL: dataset_node never came up within 60 s"; tail -30 "$log"; return 1; }
    sleep $(( MEASURE_S + 12 ))
    kill_local
    sleep 1
}

backend_value() {        # $1 = log, $2 = key — the last `stats backend` line; cumulative
    grep -h 'odometry_node' "$1" | grep -o 'stats backend .*' | tail -1 |
        awk -v key="$2" '{
            for (i = 1; i <= NF; ++i) { split($i, kv, "="); if (kv[1] == key) { v = kv[2] } }
        } END { print v }'
}

probe_value() {          # $1 = log, $2 = key
    grep -h 'odom_probe' "$1" | grep -o 'odom_probe result .*' |
        awk -v key="$2" '{
            for (i = 1; i <= NF; ++i) { split($i, kv, "="); if (kv[1] == key) { v = kv[2] } }
        } END { print v }'
}

# Mean of a node's windowed rate over the windows where it was running.
stage_rate() {           # $1 = log, $2 = node
    grep -oE "\[$2\]: stats [a-z_ ]*rate=[0-9.]+" "$1" | grep -oE '[0-9.]+$' |
        awk -v lo="$RUNNING_HZ" -v hi="$MAX_PLAUSIBLE_HZ" '
            $1 + 0 >= lo && $1 + 0 <= hi { s += $1; n++ }
            END { if (n) printf "%.3f", s / n; else print "" }'
}

ate_of() {
    "$EVO_APE" tum "$GROUND_TRUTH" "$1" -a -s 2>&1 | awk '$1 == "rmse" { print $2 }'
}

declare -A ate rate_kp rate_fusion rate_depth
echo
echo "sequence: ${SEQ} (${SEQ_SECONDS}s), ${RUNS} run(s) per arm; BA arm ba_depth_scale_sigma:=${SCALE_SIGMA}"
for i in $(seq "$RUNS"); do
    for arm in ba noba p7; do
        sigma=0.0
        case $arm in
            ba) lm=true; lba=true; sigma=$SCALE_SIGMA ;;
            noba) lm=true; lba=false ;;
            p7) lm=false; lba=false ;;
        esac
        log="$work/$arm.$i.log"
        echo "-- run ${i}, ${arm} (local_map:=${lm} local_ba:=${lba}) --"
        run_dataset "$log" "$lm" "$lba" "$work/$arm.$i.tum" "$sigma" || { echo "FAIL gate-ba"; exit 1; }
        [[ -s $work/$arm.$i.tum ]] || { note "run ${i} ${arm} wrote no trajectory"; continue; }
        ate[$arm.$i]=$(ate_of "$work/$arm.$i.tum")
        rate_depth[$arm.$i]=$(probe_value "$log" rate)
        rate_kp[$arm.$i]=$(stage_rate "$log" keypoint_node)
        rate_fusion[$arm.$i]=$(stage_rate "$log" fusion_node)
        printf '   ATE %s m; depth %s Hz, keypoints %s Hz, fusion %s Hz\n' \
            "${ate[$arm.$i]}" "${rate_depth[$arm.$i]}" "${rate_kp[$arm.$i]}" "${rate_fusion[$arm.$i]}"
        if [[ $arm == ba ]]; then
            printf '   BA: %s solves (%s refused), %s iterations, %s free + %s fixed keyframes, %s ms (p95 %s), chi2 %s -> %s, %s outliers\n' \
                "$(backend_value "$log" ba_runs)" "$(backend_value "$log" ba_refused)" \
                "$(backend_value "$log" ba_iter)" "$(backend_value "$log" ba_free)" \
                "$(backend_value "$log" ba_fixed)" "$(backend_value "$log" ba_ms)" \
                "$(backend_value "$log" ba_ms_p95)" "$(backend_value "$log" ba_chi2_before)" \
                "$(backend_value "$log" ba_chi2_after)" "$(backend_value "$log" ba_outliers)"
            # -1 is "no scale was solved", which is the right answer with the model
            # off and a failure with it on: the vertices were never built.
            printf '   depth-map scale: mean |s - 1| %s over %s free keyframes\n' \
                "$(backend_value "$log" ba_scale_dev)" "$(backend_value "$log" ba_scale_n)"
            if [[ $SCALE_SIGMA != 0.0 && $SCALE_SIGMA != 0 ]]; then
                (( $(backend_value "$log" ba_scale_n) > 0 )) ||
                    note "run ${i}: ba_depth_scale_sigma:=${SCALE_SIGMA} and no scale was solved — the model never ran"
            fi
            # --- The second false green: a BA that never runs -----------------------
            (( $(backend_value "$log" ba_runs) > 0 )) || note "run ${i}: bundle adjustment never ran"
            in_range "$(backend_value "$log" ba_iter)" 0.5 1000 ||
                note "run ${i}: $(backend_value "$log" ba_iter) iterations per solve — a solve that stops at 0 is no solve"
            in_range "$(backend_value "$log" ba_free)" "$MIN_WINDOW_FREE" 1000 ||
                note "run ${i}: $(backend_value "$log" ba_free) free keyframes per window, floor ${MIN_WINDOW_FREE} — a window of one is a pose-only refinement"
        else
            [[ $(backend_value "$log" ba_runs) == 0 ]] ||
                note "run ${i} ${arm}: the control ran $(backend_value "$log" ba_runs) solves — it is not the control"
        fi
        # Every arm: the queue deferred rather than dropped, and the thread was niced.
        [[ $(backend_value "$log" kf_dropped) == 0 ]] ||
            note "run ${i} ${arm}: $(backend_value "$log" kf_dropped) keyframes dropped — a dropped keyframe is a hole in the map"
        [[ $(backend_value "$log" niced) == true ]] ||
            note "run ${i} ${arm}: the backend thread was not niced"
    done
done

# --- Distributions ------------------------------------------------------------------
stats_of() {             # $1 = arm, $2 = assoc-array name; prints "min median max mean"
    local -n values=$2
    local arm=$1 i
    for i in $(seq "$RUNS"); do echo "${values[$arm.$i]}"; done | awk 'NF' | sort -g |
        awk '{ v[NR] = $1; s += $1 } END {
            if (!NR) { print "none none none none"; exit }
            printf "%s %s %s %.4f\n", v[1], v[int((NR + 1) / 2)], v[NR], s / NR }'
}
read -r ba_min ba_med ba_max _ < <(stats_of ba ate)
read -r noba_min noba_med noba_max _ < <(stats_of noba ate)
read -r p7_min p7_med p7_max _ < <(stats_of p7 ate)

# --- The first false green: the ATE, not the cost ------------------------------------
if [[ $ba_max == none || $noba_min == none ]]; then
    note "no ATE for one of the arms"
else
    (( $(awk -v a="$ba_max" -v b="$noba_min" 'BEGIN { print (a + 0 < b + 0) }') )) ||
        note "BA's worst ATE (${ba_max} m) is not below no-BA's best (${noba_min} m) — #11's P15 claim is that local BA lowers the ATE"
    (( $(awk -v a="$ba_med" -v b="$noba_med" 'BEGIN { print (a != b) }') )) ||
        note "BA and no-BA have the same median ATE to the last digit — the solve is not writing anything back"
fi

# --- The system: no upstream stage slower than the no-BA runs' own spread allows ---
echo
rate_rows=()
for pair in "depth:rate_depth" "keypoints:rate_kp" "fusion:rate_fusion"; do
    stage=${pair%%:*}; name=${pair#*:}
    read -r nmin _ nmax nmean < <(stats_of noba "$name")
    read -r _ _ _ bmean < <(stats_of ba "$name")
    if [[ $nmean == none || $bmean == none ]]; then
        note "${stage}: no rate measured for one of the arms — an unmeasured rate is not an unchanged one"
        continue
    fi
    floor=$(awk -v a="$nmin" -v b="$nmax" -v m="$nmean" 'BEGIN { printf "%.4f", (b - a) / m }')
    change=$(awk -v b="$bmean" -v n="$nmean" 'BEGIN { printf "%.4f", (b - n) / n }')
    bound=$(awk -v f="$floor" -v s="$RATE_SLACK" 'BEGIN { printf "%.4f", f + s }')
    rate_rows+=("$(printf '%-10s %9s %9s %8s%% %8s%%' "$stage" "$nmean" "$bmean" \
        "$(awk -v c="$change" 'BEGIN { printf "%+.1f", 100 * c }')" \
        "$(awk -v b="$bound" 'BEGIN { printf "%.1f", 100 * b }')")")
    (( $(awk -v c="$change" -v b="$bound" 'BEGIN { print (-c <= b) }') )) ||
        note "${stage} ran $(awk -v c="$change" 'BEGIN { printf "%.1f", -100 * c }')% slower with BA, against a bound of $(awk -v b="$bound" 'BEGIN { printf "%.1f", 100 * b }')% (the no-BA runs' own spread plus ${RATE_SLACK})"
done

# =============================================================================
echo
echo "================================ gate-ba ================================"
printf '%-24s %10s %10s %10s\n' "ATE Sim(3) (m)" "ba" "noba" "p7"
printf '%-24s %10s %10s %10s\n' "  min" "$ba_min" "$noba_min" "$p7_min"
printf '%-24s %10s %10s %10s\n' "  median" "$ba_med" "$noba_med" "$p7_med"
printf '%-24s %10s %10s %10s\n' "  max" "$ba_max" "$noba_max" "$p7_max"
echo "---  stage rates, mean over runs (Hz); a slowdown past the bound fails"
printf '%-10s %9s %9s %9s %9s\n' "stage" "noba" "ba" "change" "bound"
printf '%s\n' "${rate_rows[@]}"
echo "---"
if [[ $ba_max != none && $p7_min != none ]]; then
    if (( $(awk -v a="$ba_max" -v b="$p7_min" 'BEGIN { print (a + 0 < b + 0) }') )); then
        echo "against P7's tracker: every BA run beat every P7 run."
    elif (( $(awk -v a="$ba_med" -v b="$p7_med" 'BEGIN { print (a + 0 < b + 0) }') )); then
        echo "against P7's tracker: BA's median is lower, but the distributions overlap."
    else
        echo "against P7's tracker: BA does NOT beat it — median ${ba_med} m against ${p7_med} m."
    fi
fi
echo "(printed, not asserted: P15's claim is BA against no-BA. P14 found the local"
echo " map alone worse than P7, so this line is the one that says whether milestone"
echo " G has improved the trajectory the project had before it started.)"
echo "assert: every BA run below every no-BA run; the medians differ; solves ran with"
echo "        iterations > 0 and > ${MIN_WINDOW_FREE} free keyframes; no keyframe dropped; the"
echo "        backend niced; no upstream stage slower than the no-BA spread + ${RATE_SLACK}"
echo "========================================================================="

if (( fail )); then echo "FAIL gate-ba"; exit 1; fi
echo "PASS gate-ba"
