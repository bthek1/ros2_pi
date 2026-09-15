#!/usr/bin/env bash
#
# P4's toolchain gate: a plain C++ link against ONNX Runtime reaches this GPU,
# and the budget that will be asserted on depth_node can tell a GPU from a CPU.
#
# **This is not gates/depth.sh and does not pretend to be.** It says nothing about
# a node, a topic, a stamp or a metre. It closes the one claim P4 says must be
# closed *before* any ROS code is written — that the toolchain works — and it is
# a separate script because that claim stays worth re-checking after a driver
# update, an ONNX Runtime bump or a move to another machine, long after
# depth_node exists and has its own gate.
#
# Four runs, and the three after the first are all controls. That is the shape
# this project's evidence takes, for the reason gates/ipc.sh gives at length: a
# single run that comes back fast proves nothing, because nothing in it would
# have looked different had the measurement been meaningless.
#
#   A  CUDA, linked the way the build links       assert CUDA + mean <= 80 ms
#   B  the same binary, CPU execution provider    assert mean > 80 ms
#   C  the same source, linked with CMake's
#      default dtags                              assert it does NOT get CUDA
#   +  nvidia-smi, sampled while A runs           assert the GPU saw our process
#
# **B is the one that gives the 80 ms number teeth.** A threshold only means
# something if the thing it is meant to exclude actually fails it, and asserting
# that the CPU path is *slower* than the budget is how that gets checked rather
# than assumed. Without it, an 80 ms assertion on a 53 ms measurement is a number
# nobody has ever seen fail.
#
# **C is the trap, and it cost an afternoon.** libonnxruntime_providers_cuda.so is
# dlopened by libonnxruntime.so and carries no RPATH or RUNPATH of its own, and
# DT_RUNPATH — which is what CMake and every modern linker emit by default — is
# **not inherited down a dlopen chain**, while the older DT_RPATH is. So with the
# default flags the provider cannot find libcublas.so.13 sitting in the same
# directory, ONNX Runtime falls back to the CPU, and the only symptom is that
# depth is four times slower than it should be. Measured 2026-09-15, same source,
# same libraries, one linker flag apart: RUNPATH gave CPUExecutionProvider at
# 200.69 ms and RPATH gave CUDAExecutionProvider at 52.74 ms.
#
# If C ever comes back CUDA, that is not automatically good news and it is not a
# regression either — it means the loader's behaviour or ONNX Runtime's packaging
# changed under us. Re-measure before deleting the flag; do not delete the flag
# to make this gate pass.
#
#   bash tools/gates/gpu-stack.sh

# The empty option is deliberate — see tools/fetch-gpu-stack.sh on why a script
# that could be handed an argument must not let it reach the prelude.
source "$(dirname "${BASH_SOURCE[0]}")/../just-lib.sh" ""
echo "== gate-gpu-stack =="

# The budget P4 sets for depth_node's per-frame cost. Asserted here on inference
# alone, which is the floor under it: nothing depth_node adds — preprocessing,
# the reciprocal, two publishes — can make this smaller.
BUDGET_MS=80.0
# Long enough for nvidia-smi to be sampled several times while it runs.
CUDA_RUNS=200
# The CPU control is ~200 ms a run; twenty is plenty to establish it is nowhere
# near the budget and does not cost a minute of wall clock to find out.
CPU_RUNS=20

# Before arm_cleanup, always — see assert_no_session in tools/just-lib.sh: the
# cleanup handler kills this workspace's processes, so a refusal after the trap
# is armed would tear down the session it is refusing to disturb.
#
# This gate starts no ROS session, and it calls this anyway. A pipeline running
# beside it would have depth_node holding a CUDA context and several hundred
# megabytes of this 6 GB card, and the measurement would be of the two of them
# sharing — with nothing in the output saying so.
assert_no_session "bash tools/gates/gpu-stack.sh"

work=$(mktemp -d)

# The scratch directory is torn down by the *same* handler that sweeps the
# machine, rather than by a trap of its own. Two reasons, and the second is the
# one that was got wrong first: `arm_cleanup` installs handlers on EXIT and on
# INT/TERM/HUP, so a second `trap` here would either clobber them or — as the
# first version of this line did — name RETURN, which at the top level of a
# script is not a no-op so much as a trap that can never fire, leaving two
# megabytes of compiled probe in /tmp per run.
#
# kill_local's exit status is load-bearing and is preserved: it returns non-zero
# when it could not get this machine clean, and _pimesh_on_exit turns that into
# the script's exit status. A gate may not exit 0 having left something running.
gate_cleanup() {
    local rc=0
    kill_local || rc=$?
    rm -rf "$work"
    return "$rc"
}

arm_cleanup gate_cleanup

fail=0
note() { echo "FAIL: $*"; fail=1; }

# --- The stack, and the model ------------------------------------------------
#
# Both scripts verify on every run rather than only after a download, so calling
# them is the check: fetch-gpu-stack.sh asserts every required soname is present,
# that libcublas is the real 54 MB library and not the 22 kB link-time stub, and
# that the CUDA provider's dependencies all resolve; fetch-model.sh asserts the
# weights match their sha256.
echo
echo "-- the stack --"
bash "$PIMESH_WS/tools/fetch-gpu-stack.sh" || { echo "FAIL gate-gpu-stack"; exit 1; }
PREFIX=$(bash "$PIMESH_WS/tools/fetch-gpu-stack.sh" --print-prefix)

echo
echo "-- the model --"
bash "$PIMESH_WS/tools/fetch-model.sh" || { echo "FAIL gate-gpu-stack"; exit 1; }
MODEL=$(bash "$PIMESH_WS/tools/fetch-model.sh" --print-path)

# --- The driver --------------------------------------------------------------
#
# Named separately because it is the one part of the stack the fetch script
# cannot install, and "no GPU visible" and "the GPU is slow" are worth telling
# apart before any timing is read.
if ! command -v nvidia-smi >/dev/null 2>&1; then
    note "nvidia-smi is absent — there is no driver to run any of this on"
    echo "FAIL gate-gpu-stack"
    exit 1
fi
read -r GPU_NAME DRIVER < <(nvidia-smi --query-gpu=name,driver_version \
    --format=csv,noheader | awk -F', *' '{print $1 "|" $2}' | tr '|' ' ' |
    awk '{name=""; for (i=1;i<NF;i++) name=name (i>1?"_":"") $i; print name, $NF}')
echo
echo "-- the driver --"
echo "gpu    : ${GPU_NAME//_/ }"
echo "driver : ${DRIVER}"

# --- Compile, both ways ------------------------------------------------------
#
# g++ directly, not colcon: what is being tested is whether a plain C++ link
# against the tarball reaches the GPU, and putting ament, an overlay and a
# component container in front of that would be testing four things and
# reporting one.
compile() {             # $1 = output binary, $2 = extra linker flag
    g++ -O2 -std=c++17 -Wall -Wextra \
        -I"$PREFIX/include" "$PIMESH_WS/tools/gpu_probe.cpp" -o "$1" \
        -L"$PREFIX/lib" -lonnxruntime "$2" -Wl,-rpath,"$PREFIX/lib"
}

echo
echo "-- compiling the probe --"
compile "$work/probe_rpath" -Wl,--disable-new-dtags ||
    { note "the probe does not compile against $PREFIX"; echo "FAIL gate-gpu-stack"; exit 1; }
compile "$work/probe_runpath" -Wl,--enable-new-dtags ||
    { note "the control probe does not compile"; echo "FAIL gate-gpu-stack"; exit 1; }

tag_of() {              # $1 = binary -> RPATH or RUNPATH, whichever it carries
    objdump -p "$1" | awk '/R(UN)?PATH/ {print $1; exit}'
}
echo "linked with $(tag_of "$work/probe_rpath")  (the build's flags)"
echo "control with $(tag_of "$work/probe_runpath")  (CMake's default)"

[[ $(tag_of "$work/probe_rpath") == RPATH ]] ||
    note "the probe was meant to carry DT_RPATH and carries $(tag_of "$work/probe_rpath") — \
the control below is then not a control, it is the same binary twice"

probe_value() {         # $1 = file, $2 = key
    awk -F= -v key="$2" '$1 == "gpu " key {sub(/^[^=]*=/, ""); print}' "$1"
}

# --- Run A: CUDA, and an outside witness that it was really the GPU ----------
#
# The probe reports the execution provider its session was *built* with, which
# is a real signal but not proof that any arithmetic happened on the card. So
# while it runs, nvidia-smi is asked which processes hold compute contexts. That
# is the NVIDIA driver's own opinion, arrived at without going through ONNX
# Runtime at all, and it is the only claim here that does not depend on
# believing the thing being measured.
echo
echo "-- run A: CUDA execution provider, ${CUDA_RUNS} runs --"
run_for 300 "$work/probe_rpath" "$MODEL" --runs "$CUDA_RUNS" \
    >"$work/a.out" 2>"$work/a.err" &
probe_pid=$!

witness=absent
for _ in $(seq 60); do
    sleep 0.5
    kill -0 "$probe_pid" 2>/dev/null || break
    if nvidia-smi --query-compute-apps=pid,process_name,used_gpu_memory \
        --format=csv,noheader 2>/dev/null | grep -q 'probe_rpath'; then
        witness=$(nvidia-smi --query-compute-apps=pid,process_name,used_gpu_memory \
            --format=csv,noheader 2>/dev/null | grep 'probe_rpath' | head -1)
        break
    fi
done
wait "$probe_pid" || true

cuda_provider=$(probe_value "$work/a.out" provider)
cuda_mean=$(probe_value "$work/a.out" mean_ms)
cuda_p95=$(probe_value "$work/a.out" p95_ms)
cuda_min=$(probe_value "$work/a.out" min_ms)
cuda_first=$(probe_value "$work/a.out" first_ms)
model_in=$(probe_value "$work/a.out" input)
model_out=$(probe_value "$work/a.out" output)
model_shape=$(probe_value "$work/a.out" output_shape)
available=$(probe_value "$work/a.out" available)
refusal=$(probe_value "$work/a.out" refusal)

if [[ -z ${cuda_mean:-} ]]; then
    note "the probe produced no timing at all — nothing below can be read"
    tail -20 "$work/a.err"
    echo "FAIL gate-gpu-stack"
    exit 1
fi

[[ $cuda_provider == CUDAExecutionProvider ]] ||
    note "the session was built with ${cuda_provider}, not CUDAExecutionProvider${refusal:+ — $refusal}"
in_range "$cuda_mean" 0 "$BUDGET_MS" ||
    note "mean inference ${cuda_mean} ms, budget is ${BUDGET_MS} ms"
[[ $witness != absent ]] ||
    note "nvidia-smi never saw the probe holding a compute context — the timing \
may be right and the provider name may be right, and neither is the driver saying \
the GPU did the work"

# --- Run B: does the budget discriminate? ------------------------------------
echo
echo "-- run B: CPU execution provider, ${CPU_RUNS} runs (the control for the budget) --"
run_for 300 "$work/probe_rpath" "$MODEL" --cpu --runs "$CPU_RUNS" \
    >"$work/b.out" 2>"$work/b.err" || true
cpu_provider=$(probe_value "$work/b.out" provider)
cpu_mean=$(probe_value "$work/b.out" mean_ms)

if [[ -z ${cpu_mean:-} ]]; then
    note "the CPU control produced no timing, so the budget was not shown to discriminate"
else
    [[ $cpu_provider == CPUExecutionProvider ]] ||
        note "the CPU control reports ${cpu_provider} — it is not a CPU control"
    # Strictly greater than the budget: if the CPU could meet it, an assertion on
    # the budget would not be an assertion about the GPU.
    awk -v v="$cpu_mean" -v b="$BUDGET_MS" 'BEGIN { exit !(v > b) }' ||
        note "the CPU path runs at ${cpu_mean} ms, inside the ${BUDGET_MS} ms budget — \
that budget cannot tell a GPU from a CPU and run A proves nothing"
fi

# --- Run C: the linker flag, which is the whole reason this gate exists ------
echo
echo "-- run C: same source, CMake's default dtags (the documented trap) --"
run_for 300 "$work/probe_runpath" "$MODEL" --runs "$CPU_RUNS" \
    >"$work/c.out" 2>"$work/c.err" || true
runpath_provider=$(probe_value "$work/c.out" provider)
runpath_mean=$(probe_value "$work/c.out" mean_ms)
runpath_refusal=$(probe_value "$work/c.out" refusal)

if [[ -z ${runpath_mean:-} ]]; then
    note "the dtags control produced no timing"
elif [[ $runpath_provider == CUDAExecutionProvider ]]; then
    note "the DT_RUNPATH build reached CUDA, where on 2026-09-15 it could not.
      This is not a regression and it is not automatically good news: the loader's
      inheritance rules or ONNX Runtime's packaging changed under us. Re-measure
      before removing -Wl,--disable-new-dtags from the build. Do NOT remove the
      flag to make this gate pass."
fi

echo
echo "gpu                 : ${GPU_NAME//_/ }, driver ${DRIVER}"
echo "prefix              : ${PREFIX}"
echo "model               : $(basename "$MODEL")  in=${model_in} out=${model_out} ${model_shape}"
echo "providers compiled  : ${available}"
echo
echo "A  CUDA, DT_RPATH   : ${cuda_provider}  mean ${cuda_mean} ms  (assert <= ${BUDGET_MS})"
echo "     p95            : ${cuda_p95} ms   min ${cuda_min} ms   first ${cuda_first} ms (cold: context + autotune)"
echo "     driver witness : ${witness}"
echo "B  CPU control      : ${cpu_provider:-none}  mean ${cpu_mean:-?} ms  (assert > ${BUDGET_MS}: the budget must discriminate)"
echo "C  DT_RUNPATH       : ${runpath_provider:-none}  mean ${runpath_mean:-?} ms  (assert not CUDA)"
[[ -n ${runpath_refusal:-} ]] && echo "     why            : ${runpath_refusal:0:120}"

(( fail == 0 )) || { echo "FAIL gate-gpu-stack"; exit 1; }
echo "PASS gate-gpu-stack"
