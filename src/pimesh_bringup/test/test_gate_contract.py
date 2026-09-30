"""The keys a gate reads off a log line, against the keys the node writes.

**Two files that agree only because one person typed both, across a language
boundary** — the `volume_key` trap and test_dashboard_contract's subject, arriving
at #11's gates. `odometry_node` writes `stats map ...` and `stats backend ...` as
printf format strings in C++; `tools/gates/map.sh` and `tools/gates/ba.sh` pull
numbers off them by key, in awk. Rename a key on one side and the gate's parser
returns an empty string, and every way the gates then fail is a *plausible* one:

  - `in_range "" 2 1000` is false, so a renamed `local_kf_p50` reads as "the local
    map was the reference alone" — the false green the assertion exists for,
    reported against a map that was fine;
  - `(( ${culled_pts:-0} >= 1 ))` reads a missing key as zero culls;
  - the printed table shows a blank, which in a column of numbers reads as a
    figure somebody forgot rather than a key that no longer exists.

So this reads both sides as text — inelegant, and the only thing that can check
it: the reader is a shell script and cannot be linked against — and asserts every
key a gate reads is one the node writes. Each extraction is asserted to have found
a floor of keys first, because an extractor that finds nothing makes the subset
check vacuously true: `topics referenced: 0` in gates/view-configs.sh, again.
"""

import os
import re

_HERE = os.path.dirname(__file__)
_WORKSPACE = os.path.abspath(os.path.join(_HERE, '..', '..', '..'))


def _read(*parts):
    with open(os.path.join(_WORKSPACE, *parts)) as handle:
        return handle.read()


def _format_keys(source, prefix):
    """Every `key=` in the printf format that starts with `prefix`.

    The format is a run of adjacent C string literals — the compiler concatenates
    them — so this takes the literal the prefix is in and every literal that
    follows it separated only by whitespace, and stops at the first thing that is
    not one: the comma before the arguments.
    """
    start = source.find('"' + prefix)
    assert start >= 0, f'no format string starting "{prefix}" in the source'
    literal = re.compile(r'\s*"((?:[^"\\]|\\.)*)"')
    text = ''
    position = start
    while True:
        match = literal.match(source, position)
        if not match:
            break
        text += match.group(1)
        position = match.end()
    return set(re.findall(r'(\w+)=%', text))


def _shell_keys(script, reader):
    """Every key `script` passes to its `reader` function as the second argument."""
    return set(re.findall(reader + r'\s+"[^"]*"\s+([a-z_0-9]+)', script))


def _loop_keys(script):
    """The keys in map.sh's `for key in ...; do` table."""
    match = re.search(r'for key in (.*?); do', script, re.S)
    assert match, 'map.sh no longer prints its table from a `for key in` list'
    return set(match.group(1).replace('\\', ' ').split())


_ODOMETRY = _read('src', 'pimesh_frontend', 'src', 'odometry_node.cpp')
_MAP_SH = _read('tools', 'gates', 'map.sh')
_BA_SH = _read('tools', 'gates', 'ba.sh')


def test_every_key_map_sh_reads_is_one_odometry_node_writes():
    written = _format_keys(_ODOMETRY, 'stats map ')
    read = _shell_keys(_MAP_SH, 'map_value') | _loop_keys(_MAP_SH)
    # Floors, so that neither extraction can be empty and the subset trivially true.
    assert len(written) >= 25, f'only {len(written)} keys found in the stats map format'
    assert len(read) >= 20, f'only {len(read)} keys found in map.sh'
    missing = read - written
    assert not missing, (
        f'map.sh reads {sorted(missing)} off `stats map`, which odometry_node does not '
        'write — the gate would parse an empty string and report it as a failure of '
        'the map, or as a zero')


def test_every_key_ba_sh_reads_is_one_odometry_node_writes():
    written = _format_keys(_ODOMETRY, 'stats backend ')
    read = _shell_keys(_BA_SH, 'backend_value')
    assert len(written) >= 15, f'only {len(written)} keys found in the stats backend format'
    assert len(read) >= 10, f'only {len(read)} keys found in ba.sh'
    missing = read - written
    assert not missing, (
        f'ba.sh reads {sorted(missing)} off `stats backend`, which odometry_node does '
        'not write')


def test_the_probe_keys_the_new_gates_read_are_ones_odom_probe_writes():
    probe = _read('src', 'pimesh_frontend', 'probes', 'odom_probe.cpp')
    written = _format_keys(probe, 'odom_probe result frames=%')
    read = _shell_keys(_BA_SH, 'probe_value')
    assert 'rate' in read, 'ba.sh no longer reads the depth rate off odom_probe'
    assert not (read - written), f'ba.sh reads {sorted(read - written)} that odom_probe does not write'


def test_the_stage_rates_ba_sh_compares_are_logged_under_the_prefix_it_greps():
    # ba.sh's stage_rate greps "[<node>]: stats ... rate=<number>". A node whose
    # stats line stops starting with `stats rate=` drops out of the comparison,
    # and "no rate measured" is a failure — but a node that is simply not in the
    # comparison is not, so this pins the two it names.
    names = set(re.findall(r'stage_rate "\$log" ([a-z_]+)', _BA_SH))
    assert names == {'keypoint_node', 'fusion_node'}, names
    for name, path in [('keypoint_node', ('pimesh_frontend', 'src', 'keypoint_node.cpp')),
                       ('fusion_node', ('pimesh_mapping', 'src', 'fusion_node.cpp'))]:
        assert '"stats rate=' in _read('src', *path), f'{name} no longer logs `stats rate=`'


def test_the_stats_lines_cannot_be_mistaken_for_the_main_one():
    # Every existing gate parses odometry_node's line with `grep -o 'stats regime=.*'`.
    # The two new lines must not start with that, or `tail -1` returns whichever
    # was printed last — gates/keypoints.sh's `cost_mean=0.00` under an 8 ms
    # budget, arriving through a second prefix.
    assert '"stats map ' in _ODOMETRY and '"stats backend ' in _ODOMETRY
    assert not re.search(r'"stats (map|backend) [^"]*regime=', _ODOMETRY)
