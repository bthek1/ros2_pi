#!/usr/bin/env bash
#
# P18 gate (#12): the volume rebuilt at the pose graph's corrected poses.
#
# Replays TUM fr1/desk in two arms, alternating, RUNS times each:
#
#   rebuild   loop_closure:=true rebuild:=true memory_dump_dir:=...   what is measured
#   closing   loop_closure:=true rebuild:=false                       the rate control
#
# After each rebuild run, tools' `rebuild_eval` reads the memory fusion_node dumped
# after its last rebuild and scores two arms over **the same remembered frames**:
#
#   corrected   each frame at the pose graph's final correction — what the rebuild used
#   control     each frame at its odometry pose alone — no loop closure at all
#
# each against those very frames rebuilt at **motion-capture** poses, Sim(3)-aligned
# onto that arm. The depth readings are identical in all three volumes; only the
# poses differ, so what separates the arms is what the correction did to the surface.
#
# **Why not the measure #12 names.** "The median paired-surface gap before and after
# a rebuild" compares a volume with the frames it was built from, and on 2026-09-30 it
# called corrected-against-uncorrected better twice and worse twice: a drifted volume
# agrees with itself as well as a correct one. That measure is printed by fusion_node
# only with rebuild_control:=true and is not used here.
#
# **And not walk1, as #12 was written**: walk1 has no ground truth. fr1/desk has
# motion capture and ten closures a run.
#
# --- What is asserted, and the false green each answers -------------------------
#
#  1. **The corrected arm is nearer the ground-truth surface than the control** —
#     smaller gap *and* higher agreement in every run, and every corrected gap below
#     every control gap across runs.
#  2. **Every rebuild integrated its whole memory** (#12's first false green: fewer
#     frames, fewer shingles, a cleaner surface for the wrong reason), and at least one
#     rebuild ran — a run with none satisfies everything else vacuously.
#  3. **Every remembered frame was judged** against ground truth, and both arms aligned:
#     a score over a subset is a score over whatever the subset happened to be.
#  4. **The mesh after the last rebuild still has open boundary loops** (#12's second
#     false green: a sealed box is the most seductive one in this project).
#  5. **The memory stayed under MEMORY_CEILING_MB**, and the rebuild thread was niced.
#  6. **No upstream stage slower** with rebuilds than in the closing arm, beyond that
#     arm's own spread plus RATE_SLACK — ba.sh's form, for the reason mesh_node gave:
#     seconds of CPU at equal priority starve depth_node without slowing it.
#  7. **Zero TF warnings** in either arm.
#
# Nothing here touches the Pi.

source "$(dirname "${BASH_SOURCE[0]}")/../lib/just-lib.sh" --overlay
echo "== gate-rebuild =="

RUNS=${PIMESH_REBUILD_RUNS:-3}
MEMORY_CEILING_MB=256
RATE_SLACK=0.02
RUNNING_HZ=5
MAX_PLAUSIBLE_HZ=31

assert_no_session "bash tools/gates/rebuild.sh"
arm_cleanup kill_local

fail=0
note() { echo "FAIL: $*"; fail=1; }

MODEL="$PIMESH_WS/models/depth_anything_v2_small.onnx"
[[ -r $MODEL ]] || { echo "FAIL: no model at ${MODEL} — bash tools/fetch-model.sh"; exit 1; }
SEQ=$(bash "$PIMESH_WS/tools/fetch-dataset.sh" --print-path)
GROUND_TRUTH="$SEQ/groundtruth.txt"
[[ -r $GROUND_TRUTH ]] || { echo "FAIL: no dataset at ${SEQ} — bash tools/fetch-dataset.sh"; exit 1; }
EVAL="$PIMESH_WS/install/pimesh_mapping/lib/pimesh_mapping/rebuild_eval"
[[ -x $EVAL ]] || { echo "FAIL: no ${EVAL} — just build"; exit 1; }

work=$(mktemp -d)
SEQ_SECONDS=$(awk '!/^#/ && NF { if (!s) s = $1; e = $1 } END { printf "%d", e - s }' "$SEQ/rgb.txt")

run_dataset() {          # $1 = log, $2 = rebuild, $3 = dump dir or ""
    local log=$1 rebuild=$2 dump=$3 window=$(( SEQ_SECONDS + 8 ))
    # Only when there is a directory: `ros2 launch` refuses `name:=` with an empty value
    # as malformed — the launch file's own default of '' is fine, the command line's is
    # not. The first run of this gate died on the control arm for exactly that.
    local extra=()
    [[ -n $dump ]] && extra=(memory_dump_dir:="$dump")
    timeout -s INT $(( window + 60 )) ros2 launch pimesh_bringup pimesh.launch.py \
        source:=dataset_node dataset_dir:="$SEQ" odom_regime:=sixdof \
        loop_closure:=true rebuild:="$rebuild" "${extra[@]}" >"$log" 2>&1 &
    local ready=0
    for _ in $(seq 120); do
        if grep -q "dataset .*replaying at" "$log" 2>/dev/null; then ready=1; break; fi
        sleep 0.5
    done
    (( ready == 1 )) || { echo "FAIL: dataset_node never came up within 60 s"; tail -30 "$log"; return 1; }
    # Past the clip by long enough for the last rebuild to finish and mesh_node's
    # next 10 s extraction to run on the rebuilt volume.
    sleep $(( window + 18 ))
    kill_local
    sleep 1
}

eval_value() {           # $1 = eval output, $2 = key
    awk -F= -v key="$2" '$1 == key { print $2 }' "$1"
}
rebuild_value() {        # $1 = log, $2 = key — the last cumulative `stats rebuild` line
    grep -h 'fusion_node' "$1" | grep -o 'stats rebuild .*' | tail -1 |
        awk -v key="$2" '{
            for (i = 1; i <= NF; ++i) { split($i, kv, "="); if (kv[1] == key) { v = kv[2] } }
        } END { print v }'
}
stage_rate() {           # $1 = log, $2 = node: mean windowed rate while running
    grep -oE "\[$2\]: stats [a-z_ ]*rate=[0-9.]+" "$1" | grep -oE '[0-9.]+$' |
        awk -v lo="$RUNNING_HZ" -v hi="$MAX_PLAUSIBLE_HZ" '
            $1 + 0 >= lo && $1 + 0 <= hi { s += $1; n++ }
            END { if (n) printf "%.3f", s / n; else print "" }'
}
tf_warnings() { grep -cE 'TF_OLD_DATA|TF_REPEATED_DATA|[Jj]ump back in time' "$1" || true; }
lt() { (( $(awk -v a="$1" -v b="$2" 'BEGIN { print (a + 0 < b + 0) }') )); }

declare -A gap_c gap_u agree_c agree_u r_depth r_kp r_fusion
declare -a rows
echo
echo "sequence: ${SEQ} (${SEQ_SECONDS}s), ${RUNS} run(s) per arm"
for i in $(seq "$RUNS"); do
    for arm in rebuild closing; do
        log="$work/$arm.$i.log"
        dump=""
        [[ $arm == rebuild ]] && dump="$work/memory.$i"
        echo "-- run ${i}, ${arm} --"
        run_dataset "$log" "$([[ $arm == rebuild ]] && echo true || echo false)" "$dump" ||
            { echo "FAIL gate-rebuild"; exit 1; }
        r_depth[$arm.$i]=$(stage_rate "$log" depth_node)
        r_kp[$arm.$i]=$(stage_rate "$log" keypoint_node)
        r_fusion[$arm.$i]=$(stage_rate "$log" fusion_node)
        warn=$(tf_warnings "$log")
        [[ $warn == 0 ]] || note "run ${i} ${arm}: ${warn} TF warning(s)"
        printf '   depth %s Hz, keypoints %s Hz, fusion %s Hz; %s TF warning(s)\n' \
            "${r_depth[$arm.$i]}" "${r_kp[$arm.$i]}" "${r_fusion[$arm.$i]}" "$warn"
        [[ $arm == rebuild ]] || continue

        # --- 2. every rebuild integrated its whole memory -------------------------
        rebuilds=$(grep -c 'rebuild done ' "$log" || true)
        short=$(grep -o 'rebuild done .*' "$log" | awk '{
            for (k = 1; k <= NF; ++k) { split($k, kv, "="); v[kv[1]] = kv[2] }
            if (v["integrated"] != v["memory"]) n++ } END { print n + 0 }')
        (( rebuilds >= 1 )) || note "run ${i}: no rebuild ran"
        [[ $short == 0 ]] || note "run ${i}: ${short} rebuild(s) integrated fewer frames than they remembered"
        # --- 5. memory and niceness -------------------------------------------------
        mem=$(rebuild_value "$log" memory_mb)
        in_range "${mem:-x}" 0 "$MEMORY_CEILING_MB" || note "run ${i}: memory ${mem:-unmeasured} MB, ceiling ${MEMORY_CEILING_MB}"
        [[ $(rebuild_value "$log" niced) == true ]] || note "run ${i}: the rebuild thread was not niced"
        # --- 4. the mesh after the last rebuild is not sealed ------------------------
        loops_after=$(awk '/rebuild done /{ seen = NR } /\[mesh_node\]: mesh /{ if (seen && NR > seen) last = $0 }
            END { print last }' "$log" | sed -n 's/.*loops=[0-9]*->\([0-9]*\).*/\1/p')
        if [[ -z $loops_after ]]; then
            note "run ${i}: no mesh was extracted after the last rebuild — nothing to judge"
        else
            (( loops_after > 0 )) || note "run ${i}: the rebuilt mesh has 0 boundary loops — a sealed box"
        fi

        # --- 1 and 3. the outside opinion -------------------------------------------
        out="$work/eval.$i.txt"
        if ! "$EVAL" "$dump" "$GROUND_TRUTH" >"$out" 2>"$out.err"; then
            note "run ${i}: rebuild_eval could not read the dump: $(cat "$out.err")"
            continue
        fi
        sed 's/^/   /' "$out"
        for a in corrected control; do
            [[ $(eval_value "$out" ${a}_judged) == "$(eval_value "$out" frames)" ]] ||
                note "run ${i}: ${a} judged $(eval_value "$out" ${a}_judged) of $(eval_value "$out" frames) frames"
            [[ $(eval_value "$out" ${a}_aligned) == 1 ]] || note "run ${i}: ${a} arm could not be aligned to ground truth"
        done
        gap_c[$i]=$(eval_value "$out" corrected_gap_m)
        gap_u[$i]=$(eval_value "$out" control_gap_m)
        agree_c[$i]=$(eval_value "$out" corrected_agree)
        agree_u[$i]=$(eval_value "$out" control_agree)
        # -1 is the instrument saying it could not measure; not a small gap.
        if ! in_range "${gap_c[$i]}" 0 100 || ! in_range "${gap_u[$i]}" 0 100; then
            note "run ${i}: a surface gap was not measured (corrected ${gap_c[$i]}, control ${gap_u[$i]})"
        fi
        lt "${gap_c[$i]}" "${gap_u[$i]}" ||
            note "run ${i}: corrected gap ${gap_c[$i]} m is not below the control's ${gap_u[$i]} m"
        lt "${agree_u[$i]}" "${agree_c[$i]}" ||
            note "run ${i}: corrected agreement ${agree_c[$i]} is not above the control's ${agree_u[$i]}"
        rows+=("$(printf '%-4s %9s %10s %10s %10s %10s %8s %8s' "$i" "$rebuilds" \
            "${gap_c[$i]}" "${gap_u[$i]}" "${agree_c[$i]}" "${agree_u[$i]}" \
            "$(eval_value "$out" corrected_ate_m)" "$(eval_value "$out" control_ate_m)")")
    done
done

# --- 1, across runs: every corrected gap below every control gap -------------------
max_c=$(for i in $(seq "$RUNS"); do echo "${gap_c[$i]}"; done | awk 'NF' | sort -g | tail -1)
min_u=$(for i in $(seq "$RUNS"); do echo "${gap_u[$i]}"; done | awk 'NF' | sort -g | head -1)
if [[ -n $max_c && -n $min_u ]]; then
    lt "$max_c" "$min_u" || note "the worst corrected gap (${max_c} m) is not below the best control gap (${min_u} m)"
fi

# --- 6. no upstream stage slower --------------------------------------------------------
rate_rows=()
for pair in "depth:r_depth" "keypoints:r_kp" "fusion:r_fusion"; do
    stage=${pair%%:*}
    declare -n rates=${pair#*:}
    read -r cmin cmax cmean < <(for i in $(seq "$RUNS"); do echo "${rates[closing.$i]}"; done | awk 'NF' |
        awk '{ if (NR == 1 || $1 < lo) lo = $1; if (NR == 1 || $1 > hi) hi = $1; s += $1 }
             END { if (NR) printf "%s %s %.4f\n", lo, hi, s / NR; else print "none none none" }')
    rmean=$(for i in $(seq "$RUNS"); do echo "${rates[rebuild.$i]}"; done | awk 'NF { s += $1; n++ } END { if (n) printf "%.4f", s / n; else print "none" }')
    unset -n rates
    if [[ $cmean == none || $rmean == none ]]; then
        note "${stage}: no rate measured for one of the arms — an unmeasured rate is not an unchanged one"
        continue
    fi
    bound=$(awk -v a="$cmin" -v b="$cmax" -v m="$cmean" -v s="$RATE_SLACK" 'BEGIN { printf "%.4f", (b - a) / m + s }')
    change=$(awk -v r="$rmean" -v c="$cmean" 'BEGIN { printf "%.4f", (r - c) / c }')
    rate_rows+=("$(printf '%-10s %9s %9s %8s%% %8s%%' "$stage" "$cmean" "$rmean" \
        "$(awk -v c="$change" 'BEGIN { printf "%+.1f", 100 * c }')" "$(awk -v b="$bound" 'BEGIN { printf "%.1f", 100 * b }')")")
    (( $(awk -v c="$change" -v b="$bound" 'BEGIN { print (-c <= b) }') )) ||
        note "${stage} ran $(awk -v c="$change" 'BEGIN { printf "%.1f", -100 * c }')% slower with rebuilds, bound $(awk -v b="$bound" 'BEGIN { printf "%.1f", 100 * b }')%"
done

# =============================================================================
echo
echo "================================ gate-rebuild ================================"
echo "surface against the same frames rebuilt at motion-capture poses (m; agreement fraction)"
printf '%-4s %9s %10s %10s %10s %10s %8s %8s\n' run rebuilds gap_corr gap_ctrl agree_corr agree_ctrl ate_corr ate_ctrl
printf '%s\n' "${rows[@]}"
echo "--- stage rates, mean over runs (Hz); a slowdown past the bound fails"
printf '%-10s %9s %9s %9s %9s\n' stage closing rebuild change bound
printf '%s\n' "${rate_rows[@]}"
echo "assert: corrected nearer the ground-truth surface than the control in every run"
echo "        (gap and agreement) and every corrected gap below every control gap; every"
echo "        rebuild integrated its whole memory; every frame judged; the rebuilt mesh"
echo "        open; memory under ${MEMORY_CEILING_MB} MB; the thread niced; no stage slower"
echo "        than the closing arm's spread + ${RATE_SLACK}; 0 TF warnings"
echo "==========================================================================="

if (( fail )); then echo "FAIL gate-rebuild"; exit 1; fi
echo "PASS gate-rebuild"
