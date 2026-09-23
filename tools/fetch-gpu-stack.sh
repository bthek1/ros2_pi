#!/usr/bin/env bash
#
# Install the C++ GPU inference stack for depth_node. Dev box only — the Pi
# never runs inference and this script refuses to be useful there.
#
# Why this exists at all, and why it is not `apt install`:
#
#   Python's onnxruntime-gpu works on this box because pip wheels vendor the
#   CUDA runtime. A C++ build gets none of that. The ONNX Runtime GPU release
#   tarball ships libonnxruntime.so and libonnxruntime_providers_cuda.so and
#   *nothing else* — `objdump -p` on the provider lists libcudart.so.13,
#   libcublas.so.13, libcublasLt.so.13 and libcurand.so.10 as NEEDED, and none
#   of those are on this machine (measured 2026-09-15: no nvcc, no libcudart
#   anywhere in the loader cache). Only the driver is installed.
#
#   Ubuntu 26.04's multiverse does carry cuda-cudart-13-1 and friends, but apt
#   needs a password this project's scripts do not have. NVIDIA publish the same
#   libraries as public redistributable tarballs with a sha256 manifest, so the
#   whole stack installs into a user-writable prefix with no root at all — and,
#   being checksummed and version-pinned here, it is reproducible in a way an
#   `apt install` of a moving target is not. Removing it is `rm -rf` of one
#   directory.
#
# Everything lands in ONE lib directory, and that is load-bearing rather than
# tidy. libonnxruntime.so carries `RUNPATH $ORIGIN`;
# libonnxruntime_providers_cuda.so — which is dlopened, not linked — carries no
# RPATH or RUNPATH at all, and DT_RUNPATH is not inherited down a dlopen chain.
# So the only two ways it can ever find libcublas.so.13 are LD_LIBRARY_PATH and
# "sitting in the same directory as the thing that loaded it". The second needs
# no environment to be right at process start, so that is the one we use: the
# CUDA and cuDNN shared objects are installed *beside* the ONNX Runtime ones and
# resolve through $ORIGIN.
#
#   bash tools/fetch-gpu-stack.sh          # install, or verify an existing one
#   bash tools/fetch-gpu-stack.sh --force  # re-extract even if it looks complete
#
# PIMESH_GPU_PREFIX overrides the destination. Nothing else in the repo hardcodes
# it: CMake asks this script's --print-prefix for the path.

# The `""` is not noise. A sourced file with no arguments of its own sees the
# *caller's* positional parameters, and just-lib.sh parses $1 as its own option —
# so a script that takes arguments and sources the prelude bare hands its argv to
# the prelude, which exits 1 with "unknown option --print-prefix" before a line of
# this file runs. Every script here that takes an argument passes the empty option
# explicitly.
source "$(dirname "${BASH_SOURCE[0]}")/lib/just-lib.sh" ""

PREFIX=${PIMESH_GPU_PREFIX:-$HOME/.local/opt/pimesh-gpu}
LIB=$PREFIX/lib
INC=$PREFIX/include
CACHE=${PIMESH_GPU_CACHE:-$PREFIX/.cache}

# --print-prefix is how CMake and the gate find the install without repeating
# the default in three places. It must stay silent and side-effect free.
if [ "${1:-}" = "--print-prefix" ]; then
    echo "$PREFIX"
    exit 0
fi

FORCE=0
[ "${1:-}" = "--force" ] && FORCE=1

CUDA_REDIST=https://developer.download.nvidia.com/compute/cuda/redist
CUDNN_REDIST=https://developer.download.nvidia.com/compute/cudnn/redist
ORT_VER=1.30.0

# name|url|sha256
#
# Versions are pinned, not resolved from the "latest" manifest, because a build
# that silently changes its CUDA underneath a measured 80 ms budget is a build
# whose numbers mean nothing. Bump them deliberately and re-run gates/depth.sh.
#
# ONNX Runtime 1.30 is the first release to ship a CUDA 13 tarball
# (onnxruntime-linux-x64-gpu_cuda13), which is what matches the CUDA 13.1
# redistributables below and the 595.91 driver's CUDA 13.2 ceiling. cuBLAS is
# 13.2.0.9 because that is the version CUDA 13.1's own manifest points at —
# the math libraries version independently of the toolkit.
#
# nvrtc and nvjitlink are here because cuBLAS JIT-compiles kernels for shapes it
# has no precompiled path for; leaving them out gives a cuBLAS that loads and
# then fails at the first unusual GEMM, which is the worst of both worlds.
COMPONENTS=(
"onnxruntime|https://github.com/microsoft/onnxruntime/releases/download/v${ORT_VER}/onnxruntime-linux-x64-gpu_cuda13-${ORT_VER}.tgz|382d79133112388cf94ce5855789b7c9bef12bef76a08b6b277e5a317213adcd"
"cuda_cudart|$CUDA_REDIST/cuda_cudart/linux-x86_64/cuda_cudart-linux-x86_64-13.1.80-archive.tar.xz|b626f4790f46bc9324a1047f2fbcc9a42bc4a722b053056e61cc00da54ad6f32"
"libcublas|$CUDA_REDIST/libcublas/linux-x86_64/libcublas-linux-x86_64-13.2.0.9-archive.tar.xz|e23c85f0d80b0c8af5e76a87dc400a123f455e4a603524ed6219616c33283cda"
"libcurand|$CUDA_REDIST/libcurand/linux-x86_64/libcurand-linux-x86_64-10.4.1.34-archive.tar.xz|447ed93a372162db3713777c01483bca6b0c6611f40540ac061d6d3b8669d58c"
"cuda_nvrtc|$CUDA_REDIST/cuda_nvrtc/linux-x86_64/cuda_nvrtc-linux-x86_64-13.1.80-archive.tar.xz|e4250fe1b8d02fa324ecae50777ba6d23a74712a2a23eb6233fced170de876af"
"libnvjitlink|$CUDA_REDIST/libnvjitlink/linux-x86_64/libnvjitlink-linux-x86_64-13.1.80-archive.tar.xz|850cba213192691046441b5334588daa796c3dbce71253131fbf3c77e29ca18d"
"cudnn|$CUDNN_REDIST/cudnn/linux-x86_64/cudnn-linux-x86_64-9.26.0.51_cuda13-archive.tar.xz|e62c9b4af62ea130765ea5b13244c05c9452edf1690072373d193d19766d9850"
)

# The libraries the whole thing exists to provide. Checked by name at the end,
# because "the tarballs extracted" and "the loader can resolve the CUDA provider"
# are different claims and only the second one matters.
REQUIRED_SONAMES=(
    libonnxruntime.so
    libonnxruntime_providers_cuda.so
    libonnxruntime_providers_shared.so
    libcudart.so.13
    libcublas.so.13
    libcublasLt.so.13
    libcurand.so.10
    libnvrtc.so.13
    libnvJitLink.so.13
    libcudnn.so.9
)

have_all() {
    local so
    for so in "${REQUIRED_SONAMES[@]}"; do
        [ -e "$LIB/$so" ] || return 1
    done
    [ -r "$INC/onnxruntime_cxx_api.h" ] || return 1
    return 0
}

if [ "$FORCE" = 0 ] && have_all; then
    echo "fetch-gpu-stack: already installed in $PREFIX"
else
    mkdir -p "$LIB" "$INC" "$CACHE"

    for entry in "${COMPONENTS[@]}"; do
        IFS='|' read -r name url want <<<"$entry"
        archive=$CACHE/$(basename "$url")

        # Re-hash rather than trusting the file's presence: a download killed by
        # a dropped link leaves a short file with a plausible name, and the
        # symptom of using one is a linker error nobody connects to the network.
        if [ -r "$archive" ] && [ "$(sha256sum "$archive" | cut -d' ' -f1)" = "$want" ]; then
            echo "fetch-gpu-stack: $name — cached, sha256 ok"
        else
            echo "fetch-gpu-stack: $name — downloading"
            # `-C -` and a persistent .part, because cuDNN is 868 MB and this
            # link drops: measured 2026-09-15, one attempt died at 186 MB with
            # "SSL_read: unexpected eof". Restarting an 868 MB download from
            # zero on every blip is how a fetch script becomes something people
            # work around. --retry-all-errors is needed for the same reason —
            # plain --retry ignores a mid-transfer connection loss.
            curl -fL --no-progress-meter -C - \
                --retry 10 --retry-all-errors --retry-delay 3 \
                --connect-timeout 20 -o "$archive.part" "$url"
            got=$(sha256sum "$archive.part" | cut -d' ' -f1)
            if [ "$got" != "$want" ]; then
                rm -f "$archive.part"
                echo "fetch-gpu-stack: $name sha256 mismatch" >&2
                echo "  want $want" >&2
                echo "  got  $got" >&2
                exit 1
            fi
            mv "$archive.part" "$archive"
            echo "fetch-gpu-stack: $name — downloaded, sha256 ok"
        fi

        # Flatten into one prefix. The archives all have a single top-level
        # directory with lib/ and include/ under it, so the strip depth is 1 and
        # the copy is a glob rather than a recursive merge of unknown shape.
        stage=$CACHE/stage-$name
        rm -rf "$stage"
        mkdir -p "$stage"
        tar xf "$archive" -C "$stage" --strip-components=1

        # Shared objects only, and **never** the ones under lib/stubs/. The
        # static archives in the CUDA tarballs are most of their size and
        # nothing here links statically.
        #
        # The stubs exclusion is the trap this script was written wrong once.
        # Every CUDA redistributable ships a lib/stubs/ holding link-time-only
        # placeholders — libcublas.so, libcublasLt.so, and in cudart a
        # libcuda.so standing in for the driver. Flattening lib/ and lib/stubs/
        # into one directory let the 22 kB stub overwrite the 800 MB real
        # cuBLAS, and the result loaded, resolved every symbol, printed
        # "You are running using the stub version of cublas" on *stdout* where
        # nobody looks, and segfaulted on the first inference (measured
        # 2026-09-15). Note what that means for the check at the foot of this
        # script: `ldd` reporting no unresolved dependencies was true of the
        # broken install. A library that resolves is not a library that works.
        find "$stage" -path '*/stubs' -prune -o \
            -path '*/lib*' -name '*.so*' \( -type f -o -type l \) -exec cp -a {} "$LIB/" \;
        if [ -d "$stage/include" ]; then
            cp -a "$stage/include/." "$INC/"
        fi
        rm -rf "$stage"
    done
fi

# --- Verify, because extracting is not the claim -----------------------------
#
# Two checks, and the second is the one worth having. The first says the files
# are present; the second runs the actual dynamic loader over the CUDA provider
# and asserts nothing comes back "not found" — which is the exact failure that
# otherwise shows up hours later as an ONNX Runtime session quietly reporting
# CPUExecutionProvider.
missing=()
for so in "${REQUIRED_SONAMES[@]}"; do
    [ -e "$LIB/$so" ] || missing+=("$so")
done
if [ ${#missing[@]} -ne 0 ]; then
    echo "fetch-gpu-stack: FAIL — missing from $LIB:" >&2
    printf '  %s\n' "${missing[@]}" >&2
    exit 1
fi

# The stub discriminator. See the extraction comment: a stub resolves, links and
# segfaults, so it cannot be caught by asking the loader anything. Size can tell
# them apart with a margin of three orders of magnitude — the real cuBLAS is
# 54 MB of precompiled kernels (measured) and the stub is ~22 kB of empty
# symbols. The floor below is 1 MB, which is nowhere near either.
cublas_bytes=$(stat -Lc %s "$LIB/libcublas.so.13")
if [ "$cublas_bytes" -lt 1000000 ]; then
    echo "fetch-gpu-stack: FAIL — libcublas.so.13 is $cublas_bytes bytes." >&2
    echo "  That is the link-time stub from lib/stubs/, not the real library." >&2
    echo "  Re-run with --force; if it recurs the stubs exclusion has regressed." >&2
    exit 1
fi

unresolved=$(LD_LIBRARY_PATH="$LIB" ldd "$LIB/libonnxruntime_providers_cuda.so" |
    grep -i "not found" || true)
if [ -n "$unresolved" ]; then
    echo "fetch-gpu-stack: FAIL — the CUDA provider has unresolved dependencies:" >&2
    echo "$unresolved" >&2
    exit 1
fi

# libcuda.so.1 is the *driver*, not part of any redistributable, and it is the
# one dependency this script cannot install. Say so by name if it is absent,
# rather than letting it hide among the others.
if ! LD_LIBRARY_PATH="$LIB" ldd "$LIB/libonnxruntime_providers_cuda.so" |
    grep -q "libcuda.so.1 => /"; then
    echo "fetch-gpu-stack: FAIL — libcuda.so.1 does not resolve." >&2
    echo "  That is the NVIDIA driver, which this script does not install." >&2
    exit 1
fi

echo "fetch-gpu-stack: OK"
echo "  prefix   $PREFIX"
echo "  size     $(du -sh "$LIB" | cut -f1) of shared objects in lib/"
echo "  ort      $ORT_VER (gpu_cuda13)"
echo "  provider $LIB/libonnxruntime_providers_cuda.so — all dependencies resolve"
