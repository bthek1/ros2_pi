#!/usr/bin/env bash
#
# View gate: every topic a committed .rviz config references is a topic this
# workspace's nodes actually advertise.
#
# The failure this exists for is silent by construction. RViz subscribes to
# whatever name is in the config; if nothing publishes it, the display shows an
# empty panel and RViz reports "No messages received" — which looks exactly like
# a camera that is not running, a Wi-Fi link that is down, or a QoS mismatch.
# There is nothing in that panel that says "this topic does not exist and never
# did". So a renamed topic costs somebody an evening of debugging the wrong
# thing, and it costs it *later*, when the rename is no longer the obvious
# suspect. Better that it fails a script the moment the rename lands.
#
# The topic set is read from the source, not from a running graph. A gate that
# needed the pipeline up to check a config file would only ever be run when
# somebody already suspected something.

source "$(dirname "${BASH_SOURCE[0]}")/../just-lib.sh"
echo "== gate-view-configs =="

fail=0
note() { echo "FAIL: $*"; fail=1; }

# What the workspace publishes, gathered from the create_publisher calls in
# src/. Relative names are resolved into the root namespace, which is where the
# launch files put these nodes.
#
# grep over source is a blunt instrument and it is the right one here: the
# alternative is a hand-maintained list, which is a second place for the truth
# to live and the exact thing this gate is trying to stop existing.
mapfile -t published < <(
    grep -rhoP 'create_publisher<[^>]+>\(\s*"\K[^"]+' "$PIMESH_WS/src" |
        sed 's|^~/|/|; s|^\([^/]\)|/\1|' | sort -u
)

if (( ${#published[@]} == 0 )); then
    echo "FAIL: found no create_publisher calls in src/ — this gate is not looking where it thinks"
    exit 1
fi

# image_transport's convention: a display subscribing to /image_raw with
# transport `compressed` connects to /image_raw/compressed. The config names the
# base and the transport separately — writing the full name in Topic makes RViz
# look for /image_raw/compressed/compressed — so the base has to count as
# published when the suffixed topic is.
for topic in "${published[@]}"; do
    [[ $topic == */compressed ]] && published+=("${topic%/compressed}")
done
mapfile -t published < <(printf '%s\n' "${published[@]}" | sort -u)

echo "topics published by src/:"
printf '  %s\n' "${published[@]}"

mapfile -t configs < <(find "$PIMESH_WS/src" -name '*.rviz' | sort)
if (( ${#configs[@]} == 0 )); then
    echo "FAIL: no .rviz configs found under src/"
    exit 1
fi

checked=0
for config in "${configs[@]}"; do
    echo
    echo "$(realpath --relative-to="$PIMESH_WS" "$config"):"

    # Parse as YAML rather than grep for "Value:". A .rviz is YAML, and the same
    # key means different things at different depths — `Value: true` on a
    # display is its enabled flag. Only Topic.Value is a topic name.
    mapfile -t referenced < <(python3 - "$config" <<'PY'
import sys
import yaml

with open(sys.argv[1]) as fh:
    config = yaml.safe_load(fh)

for display in config.get('Visualization Manager', {}).get('Displays', []):
    topic = display.get('Topic')
    if isinstance(topic, dict) and topic.get('Value'):
        print(display.get('Name', '?'), topic['Value'])
    elif isinstance(topic, str) and topic:
        print(display.get('Name', '?'), topic)
PY
)

    # A TF display carries no topic and is not a gap in the check: /tf and
    # /tf_static are subscribed by the display's own machinery, and P0's static
    # publishers are what put anything on them.
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
            note "$(basename "$config") display '${display}' subscribes ${topic}, which nothing in src/ publishes"
        fi
    done < <(printf '%s\n' "${referenced[@]}")
done

echo
echo "configs          : ${#configs[@]}"
echo "topics referenced: ${checked}  (assert every one is published by src/)"
echo "topics published : ${#published[@]}"

(( fail == 0 )) || { echo "FAIL gate-view-configs"; exit 1; }
echo "PASS gate-view-configs"
