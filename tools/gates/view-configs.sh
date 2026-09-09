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

source "$(dirname "${BASH_SOURCE[0]}")/../just-lib.sh" --overlay
echo "== gate-view-configs =="

fail=0
note() { echo "FAIL: $*"; fail=1; }

arm_cleanup

# --- What the workspace publishes -------------------------------------------
#
# Gathered from the create_publisher calls in src/, resolved into the root
# namespace, which is where the launch files put these nodes. grep over source
# is a blunt instrument and it is the right one: the alternative is a
# hand-maintained list, which is a second place for the truth to live and the
# exact thing this gate is trying to stop existing.
mapfile -t published < <(
    grep -rhoP 'create_publisher<[^>]+>\(\s*"\K[^"]+' "$PIMESH_WS/src" |
        sed 's|^~/|/|; s|^\([^/]\)|/\1|' | sort -u
)

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
# enabled flag. Only Topic.Value is a topic name.
config_topics() {       # $1 = path to a .rviz
    python3 - "$1" <<'PY'
import sys
import yaml

with open(sys.argv[1]) as fh:
    config = yaml.safe_load(fh)

for display in config.get('Visualization Manager', {}).get('Displays', []):
    topic = display.get('Topic')
    name = display.get('Name', '?')
    if isinstance(topic, dict) and topic.get('Value'):
        print(name, topic['Value'])
    elif isinstance(topic, str) and topic:
        print(name, topic)
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

    # Wait for the node, then for its subscriptions — RViz creates the node
    # before it has finished building the displays, so the first `node info`
    # after the node appears can legitimately be empty.
    node=""
    subs=""
    for _ in $(seq 40); do
        sleep 1
        [[ -z $node ]] && node=$(timeout 10 ros2 node list 2>/dev/null | grep -m1 '^/rviz' || true)
        [[ -z $node ]] && continue
        subs=$(timeout 10 ros2 node info "$node" 2>/dev/null |
               sed -n '/Subscribers:/,/Publishers:/p' |
               awk 'NF && $1 ~ /^\// {sub(/:$/, "", $1); print $1}')
        [[ -n $subs ]] && break
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
    kill_local
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
