#!/usr/bin/env bash
#
# Three offscreen renders of a saved mesh, from fixed angles.
#
# **These PNGs are P6's evidence and the RViz window is not.** A mesh that looks
# right is the single most seductive false positive in this project — a sealed
# box with no openings looks *more* finished than a correct scan — so what closes
# the phase is a file somebody can look at later and a script can point at, not
# something a person remembers seeing.
#
# The renderer is a software rasteriser in numpy (tools/eval/mesh_render.py): no GL, no
# display, no window manager, and no Open3D — which is not installed and must not
# come back, being what forced a process boundary on the Python side.
#
#   bash tools/eval/mesh-views.sh <mesh.ply> [output-dir] [prefix]
#
# With no output directory it writes beside the mesh. tools/gates/mesh.sh calls
# this and asserts on the coverage each render reports.

# The explicit "" matters: a sourced script sees the *caller's* positional
# parameters, and just-lib.sh parses $1 as its own option — so without it, the
# mesh path lands there and it exits with "unknown option".
source "$(dirname "${BASH_SOURCE[0]}")/../lib/just-lib.sh" ""

MESH=${1:-}
[[ -n $MESH ]] || { echo "usage: bash tools/eval/mesh-views.sh <mesh.ply> [out-dir] [prefix]"; exit 1; }
[[ -r $MESH ]] || { echo "no mesh at ${MESH}"; exit 1; }

OUT_DIR=${2:-$(dirname "$MESH")}
PREFIX=${3:-$(basename "${MESH%.ply}")}

# The apt python3, explicitly. PlatformIO's venv wins `#!/usr/bin/env python3` on
# this box and has never heard of cv2; a uv-managed 3.14 wins CMake's FindPython3
# and has not heard of it either. Naming the interpreter is the only way to stop
# the coin-flip — the same reason `just build` passes -DPython3_EXECUTABLE.
/usr/bin/python3 "$PIMESH_WS/tools/eval/mesh_render.py" "$MESH" "$OUT_DIR" --prefix "$PREFIX"
