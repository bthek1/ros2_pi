#!/usr/bin/env bash
#
# Fetch Depth Anything V2 Small and verify its sha256.
#
#   bash tools/fetch-model.sh          # fetch if missing, verify either way
#   bash tools/fetch-model.sh --force  # re-download even if the file is good
#
# **The weights are never committed.** models/ and *.onnx are git-ignored, so a
# fresh clone has no model and this script is how it gets one. That makes the
# checksum the file's only identity — the same arrangement as bags/desk1, and for
# the same reason: there is nothing in the repo to compare it against.
#
# **It does not go on the Pi.** Inference is dev-box-only and the Pi is a sensor
# head; tools/pi/sync-pi.sh ships src, tools and the justfile and nothing else, so
# models/ is excluded by construction rather than by a rule somebody has to
# remember.
#
# Verifying on every run, not just after a download, is the point. A truncated
# download leaves a 60 MB file with the right name that ONNX Runtime opens and
# fails on with a protobuf parse error naming no cause, and "the model is
# present" is not the claim anyone needs.

# The `""` is not noise. A sourced file with no arguments of its own sees the
# *caller's* positional parameters, and just-lib.sh parses $1 as its own option —
# so a script that takes arguments and sources the prelude bare hands its argv to
# the prelude, which exits 1 with "unknown option --print-path" before a line of
# this file runs. Every script here that takes an argument passes the empty option
# explicitly.
source "$(dirname "${BASH_SOURCE[0]}")/lib/just-lib.sh" ""

# onnx-community's export of Depth Anything V2 Small: ViT-S/14, dynamic batch and
# spatial dims, input `pixel_values` (ImageNet-normalised NCHW float, **not** raw
# pixels), output `predicted_depth` — relative *inverse* depth, so nearer is
# larger and the units are arbitrary. 99 MB.
URL=https://huggingface.co/onnx-community/depth-anything-v2-small/resolve/main/onnx/model.onnx
SHA256=afb6a5c28f3b6bf1618c6e43f02073ef9dfdc70e937502d51603e57b0a1df10c
MODEL="$PIMESH_WS/models/depth_anything_v2_small.onnx"

FORCE=0
[ "${1:-}" = "--force" ] && FORCE=1

# --print-path is how gates/gpu-stack.sh finds the model without repeating the
# name. Silent and side-effect free.
if [ "${1:-}" = "--print-path" ]; then
    echo "$MODEL"
    exit 0
fi

verify() {
    [ -r "$MODEL" ] || return 1
    [ "$(sha256sum "$MODEL" | cut -d' ' -f1)" = "$SHA256" ]
}

if [ "$FORCE" = 0 ] && verify; then
    echo "fetch-model: already present and sha256 ok"
else
    mkdir -p "$(dirname "$MODEL")"
    echo "fetch-model: downloading Depth Anything V2 Small (99 MB)"
    # Resumable, for the same reason tools/fetch-gpu-stack.sh is: a restart from
    # zero on every dropped connection is how a fetch script becomes something
    # people work around.
    curl -fL --no-progress-meter -C - \
        --retry 10 --retry-all-errors --retry-delay 3 \
        --connect-timeout 20 -o "$MODEL.part" "$URL"
    got=$(sha256sum "$MODEL.part" | cut -d' ' -f1)
    if [ "$got" != "$SHA256" ]; then
        rm -f "$MODEL.part"
        echo "fetch-model: sha256 mismatch" >&2
        echo "  want $SHA256" >&2
        echo "  got  $got" >&2
        exit 1
    fi
    mv "$MODEL.part" "$MODEL"
fi

verify || { echo "fetch-model: FAIL — $MODEL does not match its checksum" >&2; exit 1; }

echo "fetch-model: OK"
echo "  path   $MODEL"
echo "  size   $(du -h "$MODEL" | cut -f1)"
echo "  sha256 $SHA256"
