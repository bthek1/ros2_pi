#!/usr/bin/env bash
#
# Naming gate (gh issue #14, P0): the names that resolve at *runtime* still
# resolve, and every header's location is an honest claim about whether a test
# can call it.
#
# This gate exists because the reorganisation in #14 is the one class of change
# where a C++ compiler has no opinion at all. A component's plugin string is
# looked up in the ament index at launch; a parameter finds its node by a YAML
# key; the teardown finds its processes by a path-anchored pattern; and a doc
# tells a person to run a script by name. Rename anything and all four go on
# building perfectly.
#
# The four checks, and what each would have caught:
#
#   (a) components  — a registered class that no launch list names. The failure
#       is a container that comes up without a stage and says nothing, because
#       nothing asked for the stage that is missing.
#   (b) patterns    — a kill pattern in tools/just-lib.sh that matches nothing
#       this workspace installs. tools/stragglers.sh then prints 0 on both
#       machines over a leaked camera_node holding /dev/video0, and
#       assert_no_session quietly stops refusing. Three republish processes
#       survived three and a quarter hours behind exactly this on 2026-09-12.
#   (c) names       — a doc that tells you to run something that is not there.
#   (d) headers     — a library header reaching for rclcpp, or one no test
#       names. Four helpers have been found living where no test could call
#       them (percentile, depth_mat_over, quote/number, the mesh payload
#       packing) and every time nothing was wrong *yet*.
#
# **(d) reports rather than asserts until #14's P3**, which is the phase that
# moves the node headers into nodes/. It prints its findings and its exception
# list either way, so the number cannot quietly grow in the meantime.
#
# On (b) and the prefix: `pimesh_` stays (#14 P1, abandoned deliberately), and
# the four patterns that name this workspace are globbed on it —
# `/lib/[p]imesh_[a-z]*/` matches a renamed package's directory without being
# touched. That is *why* the package renames in P2 are safe, and this check is
# what turns "we looked and they're globbed" into something that stays true.

source "$(dirname "${BASH_SOURCE[0]}")/../just-lib.sh"
echo "== gate-naming =="

fail=0
note() { echo "FAIL: $*"; fail=1; }

PY=/usr/bin/python3

# --- (a) components ---------------------------------------------------------
#
# Every RCLCPP_COMPONENTS_REGISTER_NODE in src/ must be reachable from the
# launch file: in COMPONENTS (the container), PROBE_COMPONENTS (a gate's
# instrument), or STANDALONE_NODES (its own process, matched by the node name
# the class converts to). Anything else is exempt only by being written down
# here, with the reason.

echo "-- (a) components"
"$PY" - "$PIMESH_WS" <<'PYA'
import re, sys, pathlib
ws = pathlib.Path(sys.argv[1])

# Registered here but named by no launch list, each with the reason it is not.
EXEMPT = {
    'pimesh_camera::CameraNode':
        'runs on the Pi as `ros2 run`, never composed into the dev box container',
}

reg = {}
for cpp in sorted((ws / 'src').rglob('*.cpp')):
    for m in re.finditer(r'RCLCPP_COMPONENTS_REGISTER_NODE\(\s*([A-Za-z0-9_:]+)\s*\)',
                         cpp.read_text(errors='ignore')):
        reg[m.group(1)] = cpp.relative_to(ws)

launch = (ws / 'src/pimesh_bringup/launch/pimesh.launch.py').read_text()
tuples = re.findall(r"\(\s*'([a-z0-9_]+)'\s*,\s*'([a-z0-9_]+)'\s*,\s*'([A-Za-z0-9_:]+)'\s*\)", launch)
plugins = {t[2] for t in tuples}
standalone = {(t[1], t[0]) for t in tuples if '::' not in t[2]}


def snake(cls):
    """pimesh_dashboard::DashboardNode -> ('pimesh_dashboard', 'dashboard_node')"""
    pkg, _, name = cls.partition('::')
    return pkg, re.sub(r'(?<!^)(?=[A-Z])', '_', name).lower()


bad = []
for cls, where in sorted(reg.items()):
    if cls in plugins or cls in EXEMPT or snake(cls) in standalone:
        continue
    bad.append((cls, where))

# **The package field and the plugin's namespace must agree**, and nothing but
# this says so. The container looks the class up in the ament index of the
# package the *second* field names, so a plugin string that is right beside a
# package field that is wrong fails at launch with "could not find class" and
# builds perfectly. This is not deriving one from the other — the launch file
# explains at length why it does not do that — it is checking that the two
# things somebody typed twice still say the same thing.
#
# Added 2026-09-23 after #14's P2 produced exactly that pair: splitting depth out
# left ('depth_node', 'pimesh_frontend', 'pimesh_depth::DepthNode') behind, and
# the first version of this check passed over it, because every registered class
# was named by a launch list. Ask what the gate does not touch.
mismatched = [(n, pkg, plug) for n, pkg, plug in tuples
              if '::' in plug and plug.split('::')[0] != pkg]

print(f"   {len(reg)} registered components, {len(plugins)} plugin strings in the launch lists")
for cls, reason in sorted(EXEMPT.items()):
    print(f"   exempt: {cls} — {reason}")
for cls, where in bad:
    print(f"   DANGLING: {cls} registered in {where}, named by no launch list and not exempt")
for n, pkg, plug in mismatched:
    print(f"   MISMATCH: {n} is listed under package '{pkg}' but its plugin is '{plug}' — "
          f"the container will look for it in the wrong package's index")
print(f"   {len(tuples)} launch entries, {len(mismatched)} with a package that "
      f"disagrees with the plugin namespace")
sys.exit(1 if bad or mismatched else 0)
PYA
[[ $? -eq 0 ]] || note "a registered component is reachable from no launch list"

# --- (b) patterns -----------------------------------------------------------
#
# Every PIMESH_*_PAT must match at least one command line this workspace can
# actually produce. The specimens are *derived from the tree and from
# /opt/ros*, never typed as literals — a pattern and a hand-written specimen
# that agree only because the same person typed both is the `volume_key` trap
# with extra steps.

echo "-- (b) patterns"
distro=$(basename "$(dirname "$(dirname "$(command -v ros2 2>/dev/null || echo /opt/ros/none/bin/ros2)")")")
[[ -d /opt/ros/$distro ]] || { distro=$(basename "$(ls -d /opt/ros/*/ | head -1)"); }

specimens=$(mktemp)
trap 'rm -f "$specimens"' EXIT

# Ours: every installed executable, as the absolute path a running node shows.
n_installed=0
while IFS= read -r p; do
    echo "$p --ros-args -r __node:=x" >>"$specimens"
    n_installed=$((n_installed + 1))
done < <(find -L "$PIMESH_WS/install" -type f -perm -u+x -path '*/lib/pimesh_*/*' 2>/dev/null)

# A pattern that matches nothing because nothing is built is the same silence
# this gate exists to refuse. `find -L` and not `find`: `colcon build
# --symlink-install` installs symlinks into lib/, so `-type f` without it
# reports zero over a fully built tree — which is how this assertion earned
# itself the first time it ran (2026-09-23).
if (( n_installed == 0 )); then
    note "install/ has no executables under lib/pimesh_*/ — build first (bash tools/build.sh)"
fi

# The wrappers, each assembled from a real package and a real file in the tree.
bringup_launch=$(ls "$PIMESH_WS"/src/pimesh_bringup/launch/*.launch.py | head -1)
echo "/opt/ros/$distro/bin/ros2 launch pimesh_bringup $(basename "$bringup_launch")" >>"$specimens"
cam_exe=$(find -L "$PIMESH_WS/install" -type f -perm -u+x -path '*/lib/pimesh_camera/*' -printf '%f\n' 2>/dev/null | head -1)
echo "/opt/ros/$distro/bin/ros2 run pimesh_camera ${cam_exe:-camera_node}" >>"$specimens"
echo "timeout -s INT 600 ros2 run pimesh_camera ${cam_exe:-camera_node}" >>"$specimens"
rviz_cfg=$(ls "$PIMESH_WS"/src/pimesh_bringup/rviz/*.rviz | head -1)
echo "rviz2 -d $rviz_cfg" >>"$specimens"

# Theirs: the four processes we start out of /opt/ros. The file existing is
# half the check — a pattern aimed at a path that is not installed is as dead
# as one aimed at a package that was renamed.
for rel in lib/rclcpp_components/component_container_isolated \
           lib/tf2_ros/static_transform_publisher \
           bin/ros2 \
           lib/image_transport/republish; do
    if [[ -e /opt/ros/$distro/$rel ]]; then
        case $rel in
            bin/ros2) echo "/opt/ros/$distro/bin/ros2 bag play bags/desk1" >>"$specimens" ;;
            *)        echo "/opt/ros/$distro/$rel --ros-args" >>"$specimens" ;;
        esac
    else
        note "pattern target /opt/ros/$distro/$rel is not installed"
    fi
done

n_pat=0
while IFS='=' read -r var val; do
    pat=${val//\'/}
    n_pat=$((n_pat + 1))
    if ! grep -Eq -- "$pat" "$specimens"; then
        note "$var matches no command this workspace produces: $pat"
    fi
done < <(grep -E "^PIMESH_[A-Z_]+_PAT=" "$PIMESH_WS/tools/just-lib.sh")
echo "   ${n_pat} patterns checked against $(wc -l <"$specimens") specimens (${n_installed} installed executables)"

# --- (c) names --------------------------------------------------------------
#
# **A doc that records what happened may name things that no longer exist; a
# doc that tells you what to run may not.** That line is the whole design of
# this check. docs/plans/ is the build log and the deferred register — it is
# *supposed* to contain hello-lan.sh and bags/walk1 — so it is not read here.
# What is read is the set of documents a person follows.

echo "-- (c) names"
"$PY" - "$PIMESH_WS" <<'PYC'
import re, sys, pathlib
ws = pathlib.Path(sys.argv[1])

# Read: the documents that tell a person what to run.
# Not read: docs/plans/** and docs/info/build-log.md, which are history.
targets = [ws / 'CLAUDE.md', ws / 'README.md', ws / 'justfile',
           ws / '.vscode/settings.json']
targets += [p for p in sorted((ws / 'docs/info').glob('*.md')) if p.name != 'build-log.md']

# Named on purpose before they exist, deleted and recorded as deleted, or built
# rather than committed. **Every entry is asserted to still be needed** — the
# moment the path exists, the exemption is reported as stale and has to go. An
# exemption list that cannot rot is the difference between a written reason and
# a place to put things.
EXEMPT_PATHS = {
    'tools/gpu_probe':
        'a .cpp the gpu-stack gate compiles with g++; never committed',
    'src/pimesh_hello':
        'deleted 2026-09-23; CLAUDE.md names it in the record of its deletion',
    # Gates that milestones F-I will write. Each goes when its phase lands.
    'tools/gates/trajectory.sh': 'written by #10 P11',
    'tools/gates/scale.sh': 'written by #10 P12',
    'tools/gates/map.sh': 'written by #11 P14',
    'tools/gates/ba.sh': 'written by #11 P15',
    'tools/gates/place.sh': 'written by #12 P16',
    'tools/gates/loop.sh': 'written by #12 P17',
    'tools/gates/rebuild.sh': 'written by #12 P18',
    'tools/gates/lost.sh': 'written by #13 P19',
    'tools/gates/relocalise.sh': 'written by #13 P20',
}
# Data trees are git-ignored, so a fresh clone has none of them and their
# absence says nothing about a name being right.
IGNORED_ROOTS = ('bags/', 'models/', 'calib/', 'build/', 'install/', 'log/')

pkgs = [p for p in sorted((ws / 'src').iterdir()) if p.is_dir()]


def resolves(rel):
    """A path in a doc may be workspace-relative or package-relative."""
    if (ws / rel).exists():
        return True
    return any((p / rel).exists() for p in pkgs)


# 1. Paths a reader is told to run: `bash tools/x.sh`, `just ...`, and any
#    repo-shaped path in a code span.
PATHS = re.compile(r'(?:bash\s+|`)((?:src|tools|docs|rviz|config|web|include|launch|test)'
                   r'/[A-Za-z0-9_./-]*[A-Za-z0-9_])')
bad, checked = [], set()
for t in targets:
    if not t.exists():
        continue
    text = t.read_text(errors='ignore')
    for m in PATHS.finditer(text):
        rel = m.group(1)
        # A glob or a truncated stem is prose about a family of files, not a
        # reference to one: `tools/gates/hello-*.sh` names four retired gates.
        if text[m.end():m.end() + 1] in ('*', '-'):
            continue
        if rel in checked or rel in EXEMPT_PATHS or rel.startswith(IGNORED_ROOTS):
            continue
        checked.add(rel)
        if not resolves(rel):
            bad.append((rel, t.relative_to(ws)))

# 2. Every pimesh_* token must appear somewhere in the source tree. This is
#    what separates a real node name (pimesh_container, in the launch file) or
#    a real shell function (pimesh_local_processes, in just-lib.sh) from a
#    package that was deleted.
tree = ''
for src in list((ws / 'src').rglob('*')) + list((ws / 'tools').rglob('*')) + [ws / 'justfile']:
    if src.is_file() and src.suffix in ('.py', '.cpp', '.hpp', '.sh', '.yaml', '.txt', '.xml', '.js', '') \
            and src.stat().st_size < 2_000_000:
        try:
            tree += src.read_text(errors='ignore')
        except OSError:
            pass
tree += '\n'.join(p.name for p in pkgs)

tokens, bad_tokens = set(), []
for t in targets:
    if not t.exists():
        continue
    for m in re.finditer(r'\bpimesh_[a-z_]+', t.read_text(errors='ignore')):
        tokens.add((m.group(0), t.relative_to(ws)))
for tok, where in sorted(tokens):
    if tok not in tree:
        bad_tokens.append((tok, where))

print(f"   {len(targets)} documents read, {len(checked)} paths and "
      f"{len({t for t, _ in tokens})} pimesh_* names checked")
stale_exempt = [r for r in EXEMPT_PATHS if resolves(r)]
print(f"   {len(EXEMPT_PATHS)} path exemptions, {len(stale_exempt)} of them no longer needed")
for rel in sorted(stale_exempt):
    print(f"   STALE EXEMPTION: {rel} exists now — delete its line from this gate")
for rel, where in sorted(set(bad)):
    print(f"   DANGLING: {rel} named in {where}, exists nowhere in the tree")
for tok, where in sorted(set(bad_tokens)):
    print(f"   DANGLING: {tok} named in {where}, appears nowhere in src/ or tools/")
sys.exit(1 if bad or bad_tokens or stale_exempt else 0)
PYC
[[ $? -eq 0 ]] || note "a document names something that is not in the tree"

# --- (d) headers ------------------------------------------------------------
#
# Reported, not asserted, until #14's P3 moves the component headers into
# nodes/. Turning it on is that phase's test; printing it now is what stops the
# number growing while the phase waits.

echo "-- (d) headers"
"$PY" - "$PIMESH_WS" <<'PYD'
import re, sys, pathlib
ws = pathlib.Path(sys.argv[1])

# A library header that no test names, with the reason it has none. **Every
# entry is asserted to still be needed**, like the path exemptions above: write
# a test for one of these and the gate tells you to delete its line.
EXEMPT = {
    'pimesh_depth/depth_engine.hpp':
        'a pure virtual interface — there is no behaviour here to assert. The '
        'behaviour is in the two implementations behind it, and exactly one of '
        'them is compiled per machine (ORT here, null on the Pi), so a suite '
        'covering either would break gates/test.sh\'s "same suites at both ends"',
}

rows = []
for pkg in sorted((ws / 'src').iterdir()):
    inc = pkg / 'include' / pkg.name
    if not inc.is_dir():
        continue
    tests = ''.join(p.read_text(errors='ignore')
                    for p in (pkg / 'test').rglob('*') if p.is_file())
    test_names = ' '.join(p.name for p in (pkg / 'test').rglob('*'))
    for hpp in sorted(inc.rglob('*.hpp')):
        if hpp.parent.name == 'nodes':
            continue
        stem = hpp.stem
        # The line is rclcpp — the node and executor API — not message types. A
        # header taking a sensor_msgs::msg::Image is still callable from a test,
        # and test_image_buffer is the proof.
        uses_rclcpp = bool(re.search(r'#\s*include\s*[<"]rclcpp/', hpp.read_text(errors='ignore')))
        named = (f'{stem}.hpp' in tests) or (stem in test_names)
        if uses_rclcpp or not named:
            rows.append((pkg.name, hpp.relative_to(inc), uses_rclcpp, named))

bad = [r for r in rows if f'{r[0]}/{r[1]}' not in EXEMPT]
stale = [k for k in EXEMPT if k not in {f'{r[0]}/{r[1]}' for r in rows}]
n_lib = sum(1 for pkg in (ws / 'src').iterdir()
            if (pkg / 'include' / pkg.name).is_dir()
            for h in (pkg / 'include' / pkg.name).rglob('*.hpp')
            if h.parent.name != 'nodes')

print(f"   {n_lib} library headers outside nodes/, {len(bad)} failing, "
      f"{len(EXEMPT)} exempt ({len(stale)} no longer needed)")
for k, why in sorted(EXEMPT.items()):
    print(f"   exempt: {k} — {why}")
for pkgname, rel, rclcpp, named in bad:
    why = []
    if rclcpp:
        why.append('includes rclcpp/ but is not under nodes/')
    if not named:
        why.append('no test names it')
    print(f"   FAILS: {pkgname}/{rel}: {', '.join(why)}")
for k in sorted(stale):
    print(f"   STALE EXEMPTION: {k} passes now — delete its line from this gate")
sys.exit(1 if bad or stale else 0)
PYD
[[ $? -eq 0 ]] || note "a library header reaches for rclcpp or no test names it"

# --- (e) apps and probes ----------------------------------------------------
#
# The directory says what kind of thing a file is, so that a reader does not
# have to open it. Before #14's P4 there were two spellings for a probe and no
# way to tell an executable from a component by its name at all:
# keypoint_probe_main.cpp and capture_probe_main.cpp had a main(), while
# depth_probe.cpp, odom_probe.cpp and ipc_probe.cpp are components a gate loads
# with probe:=, and all six sat in src/ beside the library code they measure.
#
# The third assertion is the one that keeps it: a main() appearing in src/ is
# how the split comes undone, one file at a time.

echo "-- (e) apps and probes"
"$PY" - "$PIMESH_WS" <<'PYE'
import re, sys, pathlib
ws = pathlib.Path(sys.argv[1])

MAIN = re.compile(r'^\s*int\s+main\s*\(', re.M)
REG = re.compile(r'RCLCPP_COMPONENTS_REGISTER_NODE\s*\(')

bad, n_apps, n_probes, n_src = [], 0, 0, 0
for pkg in sorted((ws / 'src').iterdir()):
    for cpp in sorted((pkg / 'apps').glob('*.cpp')):
        n_apps += 1
        if len(MAIN.findall(cpp.read_text(errors='ignore'))) != 1:
            bad.append((cpp.relative_to(ws), 'is under apps/ but has no main()'))
    for cpp in sorted((pkg / 'probes').glob('*.cpp')):
        n_probes += 1
        text = cpp.read_text(errors='ignore')
        if not REG.search(text):
            bad.append((cpp.relative_to(ws), 'is under probes/ but registers no component'))
        if MAIN.search(text):
            bad.append((cpp.relative_to(ws), 'is under probes/ but has a main() — it belongs in apps/'))
    for cpp in sorted((pkg / 'src').glob('*.cpp')):
        n_src += 1
        if MAIN.search(cpp.read_text(errors='ignore')):
            bad.append((cpp.relative_to(ws), 'has a main() but is in src/ — it belongs in apps/'))

print(f"   {n_apps} apps (each exactly one main), {n_probes} probes "
      f"(each exactly one registration, no main), {n_src} library sources")
for rel, why in bad:
    print(f"   MISFILED: {rel} {why}")
sys.exit(1 if bad else 0)
PYE
[[ $? -eq 0 ]] || note "a source file is not where its kind says it should be"

echo
if [[ $fail -eq 0 ]]; then
    echo "PASS gate-naming"
else
    echo "FAIL gate-naming"
fi
exit $fail
