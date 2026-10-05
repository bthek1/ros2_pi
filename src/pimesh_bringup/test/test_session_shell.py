"""The shell a session stands on: the kill patterns and two just-lib.sh helpers.

Each check is a trap #15 found by running the code, never by reading it, and each
fails *silently* — a sweep that reports clean, a teardown that kills its own
wrapper, a session that refuses to start beside itself:

- **A pattern that misses its process.** `launch_ros` execs RViz by installed
  path, so the viewer pattern, anchored on a bare `rviz2`, matched no launched
  RViz and the sweep reported a leaked window as clean.
- **A pattern that hits the wrapper.** `kill_local` does not filter by process
  group, and `PIMESH_LAUNCH_PAT` was unanchored, so it matched `tools/session.sh`,
  whose argv carries `ros2 launch pimesh_bringup …`, and SIGKILLed it mid-teardown.
  Checked here against **every** session recipe's command line, derived from the
  justfile rather than typed, so a new recipe is covered without anyone
  remembering it.
- **An exited process read as a stranger.** `pimesh_local_processes` treated an
  empty pgid — the `$(pgrep)` subshell, gone before `ps` asked — as "another
  group", and `session.sh` refused to start beside itself.
- **An empty `name:=` kept, or a full one dropped**, by `pimesh_launch_argv`.

`tools/gates/naming.sh` asserts each pattern matches *something* this workspace
produces; it cannot see what a pattern must *not* match. Patterns are POSIX EREs
for `pgrep -f`; the constructs they use (`[r]`, `[a-z]*`, `^`, `(…)?`) mean the
same in Python's `re`, and `test_the_patterns_use_only_constructs_both_read_alike`
keeps it that way.
"""

import os
import re
import subprocess

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_WS = os.path.abspath(os.path.join(_HERE, '..', '..', '..'))
_LIB = os.path.join(_WS, 'tools', 'lib', 'just-lib.sh')
_JUSTFILE = os.path.join(_WS, 'justfile')

_PAT_LINE = re.compile(r"^(PIMESH_[A-Z_]+_PAT)='([^']*)'", re.M)


@pytest.fixture(scope='module')
def patterns():
    with open(_LIB) as fh:
        found = dict(_PAT_LINE.findall(fh.read()))
    # Finding none would pass every negative check below.
    assert len(found) >= 9, f'read {len(found)} patterns from just-lib.sh — the regex is broken'
    return found


def _hits(pattern, line):
    return re.search(pattern, line) is not None


def test_the_patterns_use_only_constructs_both_read_alike(patterns):
    # Anything outside this set — \\d, \\s, lookarounds, non-greedy — either is not
    # ERE or means something else to pgrep, and the checks here would then be
    # about Python's reading of a pattern pgrep reads differently.
    for name, pat in patterns.items():
        assert re.fullmatch(r"[\w /.\[\]*^()?|:=-]*", pat), f'{name} uses {pat!r}'
        assert '\\' not in pat, f'{name} has a backslash escape'


# --- what each pattern must match -------------------------------------------

def test_the_viewer_pattern_matches_both_spellings_of_rviz(patterns):
    pat = patterns['PIMESH_VIEWER_PAT']
    cfg = '/ws/install/pimesh_bringup/share/pimesh_bringup/rviz/mesh.rviz'
    assert _hits(pat, f'rviz2 -d {cfg}'), 'RViz started by a script'
    # launch_ros's Node execs by installed path: the case the pattern once missed.
    assert _hits(pat, f'/opt/ros/lyrical/lib/rviz2/rviz2 -d {cfg} --ros-args'), \
        'RViz started by view.launch.py'


def test_the_viewer_pattern_does_not_match_what_starts_rviz(patterns):
    pat = patterns['PIMESH_VIEWER_PAT']
    for launcher in ('bash -c rviz2 -d /ws/pimesh_bringup/rviz/mesh.rviz',
                     '/usr/bin/bash -c rviz2 -d /ws/pimesh_bringup/rviz/mesh.rviz',
                     'timeout --foreground -s INT 90 rviz2 -d /ws/pimesh_bringup/x.rviz'):
        # Matching these is how kill_local once killed its own caller.
        assert not _hits(pat, launcher), launcher


def test_the_launch_pattern_matches_the_launcher(patterns):
    assert _hits(patterns['PIMESH_LAUNCH_PAT'],
                 '/usr/bin/python3 /opt/ros/lyrical/bin/ros2 launch pimesh_bringup '
                 'view.launch.py view:=mesh')


def test_the_bag_pattern_matches_a_player_and_a_recorder(patterns):
    pat = patterns['PIMESH_BAG_PAT']
    assert _hits(pat, '/usr/bin/python3 /opt/ros/lyrical/bin/ros2 bag play /ws/bags/desk1')
    # A leaked recorder is silent and fills the disk; it was invisible until #15.
    assert _hits(pat, '/usr/bin/python3 /opt/ros/lyrical/bin/ros2 bag record -s mcap -o x')


# --- what no pattern may match ----------------------------------------------

def _session_command_lines():
    """Every session recipe as the two processes it becomes, with sample values.

    `just` runs a body as `bash -euo pipefail -c <body>`, and that bash execs the
    single command in it, so both command lines exist for a moment and both are
    swept by kill_local.
    """
    with open(_JUSTFILE) as fh:
        bodies = [line.strip().lstrip('@') for line in fh if 'tools/session.sh' in line]
    lines = []
    for body in bodies:
        body = body.replace('{{ ws }}', '/ws')
        body = re.sub(r'\{\{\s*if [^}]*\}\}', '--pi', body)
        body = re.sub(r'\{\{\s*\w+\s*\}\}', 'desk1', body)
        argv = body.replace('"', '')
        lines += [f'bash -euo pipefail -c {body}', argv]
    return lines


def test_no_pattern_matches_a_session_wrapper(patterns):
    lines = _session_command_lines()
    # Every viewer and `record`; finding none would make this vacuous.
    assert len(lines) >= 2 * 9, f'{len(lines) // 2} session recipes found in the justfile'
    for line in lines:
        for name, pat in patterns.items():
            assert not _hits(pat, line), (
                f'{name} matches a session wrapper — kill_local would kill the session '
                f'from inside its own teardown:\n  {pat}\n  {line}')


# --- the two helpers ------------------------------------------------------------

def _bash(script, **env):
    return subprocess.run(['bash', '-c', f'source "{_LIB}" && {script}'],
                          capture_output=True, text=True, env={**os.environ, **env},
                          check=True).stdout


_STUBS = '''
pgrep() { echo "4242 bash /ws/tools/session.sh view-mesh --pi -- ros2 launch pimesh_bringup view.launch.py"; }
ps() { local pid=${@: -1}
       if [[ $pid == "$$" ]]; then echo 111; elif [[ $pid == 4242 ]]; then printf '%s' "$PGID_4242"; fi; }
'''


@pytest.mark.parametrize('pgid, reported', [
    ('', False),     # gone before ps asked: the $(pgrep) subshell
    ('111', False),  # the caller's own group
    ('999', True),   # a process of another session: the guard's whole job
])
def test_local_processes_reports_only_live_processes_of_other_groups(pgid, reported):
    out = _bash(_STUBS + 'pimesh_local_processes', PGID_4242=pgid)
    assert ('4242 ' in out) == reported, f'pgid {pgid!r}: {out!r}'


def test_launch_argv_drops_only_empty_name_assignments():
    words = ['ros2', 'launch', 'pimesh_bringup', 'view.launch.py', 'view:=mesh',
             'bag:=', 'seconds:=600', 'title:=a b', 'x:==', ':=', 'odom_regime:=']
    out = subprocess.run(['bash', '-c', f'source "{_LIB}" "" && pimesh_launch_argv "$@"', '_',
                          *words], capture_output=True, text=True, check=True).stdout
    assert out.splitlines() == ['ros2', 'launch', 'pimesh_bringup', 'view.launch.py',
                                'view:=mesh', 'seconds:=600', 'title:=a b', 'x:==', ':=']
