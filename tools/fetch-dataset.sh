#!/usr/bin/env bash
#
# Fetch a public RGB-D dataset with ground truth, and verify it.
#
#   bash tools/fetch-dataset.sh                    # fetch if missing, verify either way
#   bash tools/fetch-dataset.sh --print-path       # where the sequence is, silently
#   bash tools/fetch-dataset.sh --force            # re-download even if it is good
#
# **This is the only thing in the project that has an opinion about where the
# camera actually was.** Every number P0-P10 measured about the pose came out of
# the pipeline that produced the pose — a reprojection residual, a paired-surface
# gap — and P7 is the proof of how far that gets you: the rotation had been
# composed inverted since P3 and *every* internal number describing it was
# correct. TUM fr1/desk carries a motion-capture trajectory recorded by something
# that has never heard of this repository, which is the whole point of it.
#
# **The three hashes, and why there are three.** fetch-model.sh pins one file
# because the model *is* one file. Here the tarball is the download and the
# extracted tree is what gets read, and the two can disagree — an interrupted
# `tar`, a half-deleted directory, an edit. So:
#
#   - the tarball's sha256 says the download is the sequence TUM published;
#   - `rgb.txt`'s says the frame list has not moved under us;
#   - **`groundtruth.txt`'s says the truth has not**, which is the one that would
#     otherwise be unfalsifiable. Every figure tools/gates/trajectory.sh prints is
#     measured against that file. A corrupted or substituted one does not produce
#     an error; it produces an ATE, and a plausible one.
#
# **Not on the Pi.** tools/pi/sync-pi.sh ships src, tools and the justfile; this
# writes to ~/.local/share, which is outside the workspace by construction — the
# same arrangement as models/ and for the same reason. The Pi is a sensor head
# and a dataset is what you use when you are not using the sensor.

# The `""` is not noise: a sourced file with no arguments of its own sees the
# *caller's* positional parameters, and just-lib.sh parses $1 as its own option.
# See the same line in tools/fetch-model.sh.
source "$(dirname "${BASH_SOURCE[0]}")/lib/just-lib.sh" ""

# TUM RGB-D fr1/desk: 613 colour frames at 30 Hz over 20.4 s, 640x480, a
# hand-held sweep over an office desk, with a 100 Hz motion-capture trajectory
# alongside. Chosen over every other sequence in that dataset for one reason
# beyond its size: the predecessor has a number on **this** sequence to compare
# against — ATE 0.163 m, falling to 0.089 m with loop closure — so the first
# figure this project produces lands beside one somebody already measured.
#
# The URL 302s to webshare.cvg.cit.tum.de; curl -L follows it. Pinned to the
# canonical cvg.cit.tum.de path rather than to the redirect target, because the
# redirect is TUM's to change and the path is what their documentation gives.
URL=https://cvg.cit.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_desk.tgz
SHA256=e983d6830916e66dc4a46a71368046b149b283de87769690e7aa4e0b9483530c
SEQUENCE=rgbd_dataset_freiburg1_desk

# The extracted tree, checked by content and not by existence. An empty
# directory with the right name is what an interrupted `tar` leaves behind.
RGB_TXT_SHA256=d1bc510ecca08540e03be8df55af8857753b614d8a5c38526bc669ce6c802284
GT_TXT_SHA256=48c4fa61f5d78310ea579f404d31d296738c4210c19c2bdac4a5cdf6bff50831
RGB_FRAMES=613

# ~/.local/share, not models/ and not bags/. It is not this project's data — it
# is somebody else's, it is 344 MB compressed and 680 MB unpacked, and a second
# checkout of this repo should find it already here rather than fetch it again.
ROOT=${PIMESH_DATASETS:-$HOME/.local/share/pimesh-datasets}
TARBALL="$ROOT/$SEQUENCE.tgz"
SEQ_DIR="$ROOT/$SEQUENCE"

# --print-path is how tools/gates/trajectory.sh finds the sequence without
# repeating its name. Silent, side-effect free, and it prints the path whether or
# not anything is there — "no such directory" from the caller is a better message
# than a blank line from here.
if [ "${1:-}" = "--print-path" ]; then
    echo "$SEQ_DIR"
    exit 0
fi

FORCE=0
[ "${1:-}" = "--force" ] && FORCE=1

sha_of() { sha256sum "$1" 2>/dev/null | cut -d' ' -f1; }

verify_tarball() {
    [ -r "$TARBALL" ] || return 1
    [ "$(sha_of "$TARBALL")" = "$SHA256" ]
}

# Three claims about the unpacked tree, and the failure message names which one
# broke. `find ... | wc -l` rather than `ls | wc -l` so a filename with a newline
# in it cannot inflate the count — TUM's are timestamps, but a count is only
# evidence if nothing can pad it.
verify_tree() {
    [ -d "$SEQ_DIR" ] || { tree_why="no such directory"; return 1; }
    local got
    got=$(sha_of "$SEQ_DIR/rgb.txt")
    [ "$got" = "$RGB_TXT_SHA256" ] || { tree_why="rgb.txt sha256 $got, want $RGB_TXT_SHA256"; return 1; }
    got=$(sha_of "$SEQ_DIR/groundtruth.txt")
    [ "$got" = "$GT_TXT_SHA256" ] || { tree_why="groundtruth.txt sha256 $got, want $GT_TXT_SHA256"; return 1; }
    got=$(find "$SEQ_DIR/rgb" -maxdepth 1 -name '*.png' -printf . 2>/dev/null | wc -c)
    [ "$got" = "$RGB_FRAMES" ] || { tree_why="rgb/ holds $got png, want $RGB_FRAMES"; return 1; }
    tree_why=
}

tree_why=
if [ "$FORCE" = 0 ] && verify_tree; then
    echo "fetch-dataset: $SEQUENCE already unpacked and verified"
else
    if [ "$FORCE" = 0 ] && verify_tarball; then
        echo "fetch-dataset: tarball present and sha256 ok (tree: ${tree_why:-absent})"
    else
        mkdir -p "$ROOT"
        echo "fetch-dataset: downloading $SEQUENCE (344 MB)"
        # Resumable, like tools/fetch-gpu-stack.sh and for the same reason: a
        # restart from zero on every dropped connection is how a fetch script
        # becomes something people work around.
        curl -fL --no-progress-meter -C - \
            --retry 10 --retry-all-errors --retry-delay 3 \
            --connect-timeout 20 -o "$TARBALL.part" "$URL"
        got=$(sha_of "$TARBALL.part")
        if [ "$got" != "$SHA256" ]; then
            rm -f "$TARBALL.part"
            echo "fetch-dataset: sha256 mismatch" >&2
            echo "  want $SHA256" >&2
            echo "  got  $got" >&2
            exit 1
        fi
        mv "$TARBALL.part" "$TARBALL"
    fi

    echo "fetch-dataset: unpacking into $ROOT"
    # Into a fresh directory. Unpacking over a partial tree would leave whatever
    # was already wrong in place and still satisfy every check below, because
    # tar only writes the entries it has.
    rm -rf "$SEQ_DIR"
    tar xzf "$TARBALL" -C "$ROOT"
fi

# Verified on every run, not only after a fetch — the same rule fetch-model.sh
# follows. "The dataset is present" is not the claim anybody needs; the claim is
# that the frames and the truth are the ones these hashes were taken from.
verify_tree || {
    echo "fetch-dataset: FAIL — $SEQ_DIR does not match ($tree_why)" >&2
    echo "  re-fetch with: bash tools/fetch-dataset.sh --force" >&2
    exit 1
}

echo "fetch-dataset: OK"
echo "  sequence   $SEQ_DIR"
echo "  frames     $RGB_FRAMES colour at 640x480, ~30 Hz over 20.4 s"
echo "  truth      groundtruth.txt, 2335 motion-capture poses, TUM format"
echo "  sha256     $SHA256 (tarball)"
