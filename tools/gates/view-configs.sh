#!/usr/bin/env bash
#
# View gate: every topic a committed .rviz config names is a topic this
# workspace publishes, **and RViz actually subscribes to it**.
#
# Two stages, and the second one exists because the first was not enough. The
# failure this gate is for is silent by construction: RViz subscribes to
# whatever name is in the config, and if nothing publishes it the display shows
# an empty panel that looks exactly like a camera that is not running, a Wi-Fi
# link that is down, or a QoS mismatch. Nothing in the panel, the log, or the
# display's status says "this topic does not exist".
#
# **Measured 2026-09-09: the static half passed over a config that showed
# nothing.** camera.rviz named `Topic.Value: /image_raw` with a
# `Transport Hint: compressed` key beside it — which is how the display works in
# ROS 1 and not how it works here. rviz_default_plugins infers the transport
# *from the topic name* (displays/image/get_transport_from_topic.cpp); there is
# no "Transport Hint" property on the Image display at all, so RViz ignored the
# unknown key, inferred `raw` from the name, and subscribed to `/image_raw`,
# which nothing publishes. `ros2 topic info -v /image_raw/compressed` reported
# **Subscription count: 0** with the window open and the camera streaming at
# 59 Hz.
#
# And this gate said PASS, because it had been written to treat a base name as
# published whenever the `/compressed` form was — a leniency invented to
# describe a mechanism that does not exist here. It was not a check with a gap
# in it; it was a check whose one special case was precisely the bug.
#
# **Stage 1's exactness is what catches that, and stage 2 does not.** This was
# tested rather than assumed: with the broken config restored, stage 1 reports
# `/image_raw NOT PUBLISHED` and stage 2 reports `rviz subscribed` — because
# RViz *did* subscribe, to exactly the name it was given, and being given a
# useless name is not something RViz has an opinion about. Saying otherwise
# would repeat the mistake this gate is here to document.
#
# So stage 2 earns its place on a narrower claim: that rviz2 can load this file
# at all, and comes up subscribed to the literal topic it names. That is what
# fails when a display class is renamed by a distro upgrade, when the YAML is
# malformed enough for RViz to skip a display, or when RViz rewrites a name on
# its way to the graph. It is a real check and it is not the one that would
# have saved the grey panel.

source "$(dirname "${BASH_SOURCE[0]}")/../lib/just-lib.sh" --overlay
echo "== gate-view-configs =="

fail=0
note() { echo "FAIL: $*"; fail=1; }

# Before arm_cleanup, always — see assert_no_session in tools/lib/just-lib.sh:
# the cleanup handler kills this workspace's processes, so a refusal after the
# trap is armed would tear down the session it is refusing to disturb.
assert_no_session "bash tools/gates/view-configs.sh"

arm_cleanup

# --- What the workspace publishes -------------------------------------------
#
# Gathered from the create_publisher calls in src/, resolved into the root
# namespace, which is where the launch files put these nodes. Reading the source
# is a blunt instrument and it is the right one: the alternative is a
# hand-maintained list, which is a second place for the truth to live and the
# exact thing this gate is trying to stop existing.
#
# **A plain grep for a string literal was not enough, and the gap was invisible.**
# Half the publishers in this workspace do not name their topic in the call —
# they pass a variable holding a *parameter's* default, which is how a topic
# becomes configurable. `create_publisher<Image>(depth_topic, qos)` has no topic
# in it to find. So a grep for literals reported `/image_raw`, `/depth` and
# `/depth/rgb` as published by nothing at all, and any config naming one of them
# would have failed stage 1 for a reason that has nothing to do with the config.
#
# It resolves the variable back to the `declare_parameter` it came from, and
# **refuses to carry on if it cannot**: a publisher this gate could not read is a
# hole in its coverage, and a hole that says nothing is how the last version of
# this gate passed over a display that showed an empty panel for a week.
publisher_topics=$(/usr/bin/python3 - "$PIMESH_WS/src" <<'PUBS'
import pathlib
import re
import sys

root = pathlib.Path(sys.argv[1])

# create_publisher<T>("…")  — the topic named in the call.
LITERAL = re.compile(r'create_publisher<[^>]+>\(\s*"([^"]+)"')
# create_publisher<T>(some_variable, …) — the topic named somewhere else.
VARIABLE = re.compile(r'create_publisher<[^>]+>\(\s*([A-Za-z_]\w*)\s*,')

topics = set()
unresolved = []

for path in sorted(root.rglob('*.cpp')):
    text = path.read_text()
    for match in LITERAL.finditer(text):
        topics.add(match.group(1))
    for match in VARIABLE.finditer(text):
        name = match.group(1)
        # `const std::string x = declare_parameter("x_topic", std::string("/x"), …)`
        # The default is the topic the pipeline actually runs on: config/pimesh.yaml
        # sets these to the same values, and a launch that overrode one would be
        # running a topic layout no committed .rviz describes anyway.
        declared = re.search(
            r'\b' + re.escape(name) +
            r'\s*=\s*declare_parameter\s*(?:<[^>]*>)?\(\s*"[^"]*"\s*,'
            r'\s*std::string\(\s*"([^"]+)"',
            text)
        if declared:
            topics.add(declared.group(1))
        else:
            unresolved.append(f'{path.name}: create_publisher(&{name}, …)')

if unresolved:
    print('UNRESOLVED', file=sys.stderr)
    for entry in unresolved:
        print(' ', entry, file=sys.stderr)
    sys.exit(1)

for topic in sorted(topics):
    # Into the root namespace, which is where the launch files put these nodes.
    if topic.startswith('~/'):
        topic = topic[1:]
    elif not topic.startswith('/'):
        topic = '/' + topic
    print(topic)
PUBS
) || {
    echo "FAIL: a create_publisher call names a topic this gate could not resolve"
    echo "      (listed above). Stage 1 would report that topic as unpublished,"
    echo "      so the resolver in this gate has to learn the new spelling —"
    echo "      silently skipping it is how a display ends up checked by nothing."
    exit 1
}
mapfile -t published <<<"$publisher_topics"

if (( ${#published[@]} == 0 )); then
    echo "FAIL: found no create_publisher calls in src/ — this gate is not looking where it thinks"
    exit 1
fi

echo "topics published by src/:"
printf '  %s\n' "${published[@]}"

mapfile -t configs < <(find "$PIMESH_WS/src" -name '*.rviz' | sort)
if (( ${#configs[@]} == 0 )); then
    echo "FAIL: no .rviz configs found under src/"
    exit 1
fi

# Parse as YAML, not with grep for "Value:". A .rviz is YAML and the same key
# means different things at different depths — `Value: true` on a display is its
# enabled flag. Only a topic property's Value is a topic name.
#
# **Two spellings, because RViz has two.** A modern display carries a `Topic`
# group — a dict whose `Value` is the name, with the QoS beside it. An older one
# carries a bare `RosTopicProperty` named for what it is: `Depth Map Topic` and
# `Color Image Topic` on DepthCloud, which is the display milestone C added. A
# parser that knew only the first found *no topics at all* on that display and
# skipped it in silence, which is this gate's own recurring failure mode — it
# checked what it could see and said nothing about the rest. Any key ending in
# "Topic" is one.
config_topics() {       # $1 = path to a .rviz
    /usr/bin/python3 - "$1" <<'PY'
import sys
import yaml

with open(sys.argv[1]) as fh:
    config = yaml.safe_load(fh)


def topic_properties(display):
    """Every (property name, topic) this display names."""
    for key, value in display.items():
        if key != 'Topic' and not key.endswith(' Topic'):
            continue
        if isinstance(value, dict) and value.get('Value'):
            yield key, value['Value']
        elif isinstance(value, str) and value:
            yield key, value


for display in config.get('Visualization Manager', {}).get('Displays', []):
    name = display.get('Name', '?')
    for key, topic in topic_properties(display):
        # The display's name alone is ambiguous once one display names two
        # topics, so the property is part of the label. **No spaces in it**: the
        # caller reads these lines with `read -r display topic`, so a label like
        # "DepthCloud/Color Image" would split and hand it "Image" as the topic —
        # which it then reported as unpublished, in a message naming a topic
        # nobody had written anywhere.
        label = name if key == 'Topic' else name + '/' + key[:-len(' Topic')].replace(' ', '')
        print(label, topic)
PY
}

# --- Stage 1: the names exist -----------------------------------------------
#
# Exact match. A display's topic is the *full* name including any transport
# suffix, because that suffix is what tells RViz which transport to use.

echo
echo "-- stage 1: every .rviz topic is published by src/ (exact match) --"
checked=0
for config in "${configs[@]}"; do
    echo "$(realpath --relative-to="$PIMESH_WS" "$config"):"
    mapfile -t referenced < <(config_topics "$config")

    # A TF display carries no topic and that is not a gap: /tf and /tf_static
    # are subscribed by the display's own machinery.
    if (( ${#referenced[@]} == 0 )); then
        note "$(basename "$config") references no topics at all — is the parse right?"
        continue
    fi

    while read -r display topic; do
        [[ -z ${topic:-} ]] && continue
        checked=$(( checked + 1 ))
        if printf '%s\n' "${published[@]}" | grep -qx -- "$topic"; then
            printf '  %-12s %-34s ok\n' "$display" "$topic"
        else
            printf '  %-12s %-34s NOT PUBLISHED\n' "$display" "$topic"
            note "$(basename "$config") display '${display}' names ${topic}, which nothing in src/ publishes"
        fi
    done < <(printf '%s\n' "${referenced[@]}")
done

# --- Stage 2: RViz agrees ----------------------------------------------------
#
# Start each config for real and ask RViz what it subscribed to. No publisher
# and no Pi are needed — a subscription is in the ROS graph whether or not
# anything is publishing to it — so this costs one rviz2 startup per config and
# needs no hardware. It is the only check here that can see a display silently
# reinterpreting the name it was given.

echo
echo "-- stage 2: RViz subscribes to those topics when the config is loaded --"

if [[ -z ${DISPLAY:-}${WAYLAND_DISPLAY:-} ]]; then
    echo "FAIL: no display available, so RViz cannot be started and stage 2 cannot run."
    echo "      This gate does not report PASS on a check it did not perform."
    exit 1
fi

# rviz2 renders through GLX on this Wayland session.
export QT_QPA_PLATFORM=xcb

for config in "${configs[@]}"; do
    mapfile -t referenced < <(config_topics "$config")
    (( ${#referenced[@]} == 0 )) && continue

    run_for 45 rviz2 -d "$config" >/dev/null 2>&1 &

    # Wait for the node, then for **every** topic this config names — not for the
    # first subscription to appear.
    #
    # **Stopping at the first one is a race, and it lost.** RViz creates the node
    # before it has finished building the displays, and it builds them one at a
    # time: measured 2026-09-15, a DepthCloud display's depth subscription was in
    # the graph several seconds before its colour one, so a loop that broke on
    # `subs` being non-empty read the list mid-construction and reported
    # `/depth/rgb` as RVIZ DID NOT SUBSCRIBE against a config that was completely
    # correct. Given another ten seconds the same rviz2 had all three (the
    # camera_info DepthCloud derives from the depth topic's namespace included).
    #
    # So the loop ends when the answer is complete or when the time is up, and a
    # timeout leaves the last list it saw to be reported against — which is the
    # real failure, reported honestly, rather than a snapshot of a display that
    # had not been built yet.
    node=""
    subs=""
    for _ in $(seq 40); do
        sleep 1
        [[ -z $node ]] && node=$(timeout 10 ros2 node list 2>/dev/null | grep -m1 '^/rviz' || true)
        [[ -z $node ]] && continue
        subs=$(timeout 10 ros2 node info "$node" 2>/dev/null |
               sed -n '/Subscribers:/,/Publishers:/p' |
               awk 'NF && $1 ~ /^\// {sub(/:$/, "", $1); print $1}')
        [[ -n $subs ]] || continue
        missing=0
        while read -r _ topic; do
            [[ -z ${topic:-} ]] && continue
            grep -qx -- "$topic" <<<"$subs" || missing=1
        done < <(printf '%s\n' "${referenced[@]}")
        (( missing == 0 )) && break
    done

    echo "$(realpath --relative-to="$PIMESH_WS" "$config"):"
    if [[ -z $node ]]; then
        note "rviz2 never appeared in the graph for $(basename "$config")"
    elif [[ -z $subs ]]; then
        note "$node subscribed to nothing at all with $(basename "$config") loaded"
    else
        while read -r display topic; do
            [[ -z ${topic:-} ]] && continue
            if grep -qx -- "$topic" <<<"$subs"; then
                printf '  %-12s %-34s rviz subscribed\n' "$display" "$topic"
            else
                printf '  %-12s %-34s RVIZ DID NOT SUBSCRIBE\n' "$display" "$topic"
                note "RViz loaded '${display}' but did not subscribe to ${topic} — it subscribed to [$(tr '\n' ' ' <<<"$subs")]"
            fi
        done < <(printf '%s\n' "${referenced[@]}")
    fi

    # SIGINT, not the pattern kill, and then the pattern kill as a backstop.
    # rviz2 segfaults on SIGTERM — measured 2026-09-09, "Segmentation fault
    # (core dumped)" on every kill_local — and while that happens after the
    # measurement and cannot change the result, a gate that drops a core file
    # per run is teaching whoever reads its output to ignore a crash. SIGINT is
    # also the path `just view-camera` actually takes, so this exercises the
    # shutdown a person gets rather than one only the gate ever sees.
    pkill -INT -f "$PIMESH_VIEWER_PAT" 2>/dev/null || true
    for _ in $(seq 10); do
        pgrep -f "$PIMESH_VIEWER_PAT" >/dev/null 2>&1 || break
        sleep 1
    done
    # **Reported, not left to `set -e`.** `kill_local` returns non-zero when it
    # could not get the machine clean, and this script runs under `set -euo
    # pipefail` — so a bare call here ends the gate *silently*, with exit 1, no
    # FAIL line and no indication which config it had reached. Seen exactly that
    # way on 2026-09-15: stage 2 stopped after the first config having printed
    # nothing about why. A teardown that could not finish is worth a message of
    # its own; it is not worth discarding the two configs still to check.
    kill_local || note "could not tear down rviz2 after $(basename "$config")"
    sleep 2
done

cleanup_both

echo
echo "configs          : ${#configs[@]}"
echo "topics referenced: ${checked}  (assert every one is published by src/)"
echo "topics published : ${#published[@]}"
echo "live check       : RViz's own subscriber list, per config"

(( fail == 0 )) || { echo "FAIL gate-view-configs"; exit 1; }
echo "PASS gate-view-configs"
