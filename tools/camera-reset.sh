#!/usr/bin/env bash
#
# Put the camera's persistent V4L2 controls back to a known baseline, and print
# every one of them current-vs-default.
#
# **V4L2 controls live inside the camera, not in the driver.** They survive the
# process that set them, the reboot after it, and being unplugged. A benchmark
# that left a manual exposure behind makes every later session black; a
# `focus_absolute` left at 250 makes every later mesh soft. Neither looks like a
# camera problem from inside ROS — they look like a bug in whatever code ran
# next. So this is not a convenience: it is the step that makes a frame-rate or
# an image measurement mean anything, and tools/gates/capture.sh runs it before
# it measures.
#
# The one control that is not simply "put it back to its default" is
# `exposure_dynamic_framerate`. The C922 reports its default as 0 and powers on
# with it set to 1, which trades frame rate for exposure in indoor light and
# costs about 10 fps — 18-21 fps instead of 42-60 at true 720p MJPEG. Resetting
# to defaults would therefore leave it *on* half the time, so it is forced to 0
# explicitly and separately.
#
# Runs against the Pi over SSH, because that is where the camera is. Prints
# `camera-reset <key>=<value>` lines for the gate on top of the human table.

source "$(dirname "${BASH_SOURCE[0]}")/just-lib.sh"

DEVICE=${1:-/dev/video0}

# Controls with flags=inactive are gated behind an auto mode — exposure_time_
# absolute under auto_exposure, white_balance_temperature under
# white_balance_automatic — and the driver refuses to write them. That refusal
# is correct behaviour and not a failure to report; the auto control above them
# is the one that matters, and it is reset here.
parse_controls() {      # stdin = v4l2-ctl --list-ctrls output
    awk '
        /^[[:space:]]*[a-z_]+ 0x[0-9a-f]+/ {
            name = $1
            value = ""; default_ = ""; inactive = 0; readonly = 0
            for (i = 1; i <= NF; i++) {
                if ($i ~ /^value=/)   { value = substr($i, 7) }
                if ($i ~ /^default=/) { default_ = substr($i, 9) }
                if ($i ~ /inactive/)  { inactive = 1 }
                if ($i ~ /read-only/) { readonly = 1 }
            }
            if (value != "" && default_ != "") {
                print name, default_, value, inactive, readonly
            }
        }
    '
}

echo "== camera-reset =="
echo "device           : ${DEVICE} on ${PI}"

before=$(pi_run "v4l2-ctl -d $DEVICE --list-ctrls") || {
    echo "FAIL: cannot read controls from ${DEVICE} on ${PI}"
    exit 1
}

# Build the set list. Only controls that are actually off their default get
# written: a write to a UVC control is a USB round trip, and writing thirteen of
# them when one has moved turns a diagnostic into a stall.
sets=()
while read -r name default_ value inactive readonly; do
    [[ $inactive == 1 || $readonly == 1 ]] && continue
    [[ $value == "$default_" ]] && continue
    sets+=("${name}=${default_}")
done < <(parse_controls <<<"$before")

# ...and this one unconditionally, because its default is a lie about what the
# camera powers on with.
sets+=("exposure_dynamic_framerate=0")

# One at a time, and tolerating failure: a control can become unwritable between
# the read and the write (something else grabbed the device, an auto mode
# changed underneath), and one stubborn control must not stop the other twelve
# from being reset.
refused=0
for pair in "${sets[@]}"; do
    if ! pi_run "v4l2-ctl -d $DEVICE --set-ctrl ${pair}" >/dev/null 2>&1; then
        echo "  refused: ${pair}"
        refused=$(( refused + 1 ))
    fi
done

after=$(pi_run "v4l2-ctl -d $DEVICE --list-ctrls")

# The table. Every control, what it is now, what the driver calls its default,
# and whether those agree — printed whether or not anything was changed,
# because the value of this script is as much "show me the camera's state" as
# it is "fix it".
echo
printf '  %-30s %10s %10s  %s\n' control current default ''
off_default=0
while read -r name default_ value inactive readonly; do
    mark=""
    if [[ $inactive == 1 ]]; then
        mark="(inactive — gated behind an auto mode)"
    elif [[ $readonly == 1 ]]; then
        mark="(read-only)"
    elif [[ $value != "$default_" ]]; then
        mark="<-- off default"
        off_default=$(( off_default + 1 ))
    fi
    printf '  %-30s %10s %10s  %s\n' "$name" "$value" "$default_" "$mark"
done < <(parse_controls <<<"$after")

# The exposure mode, by name, because no frame rate from this camera means
# anything without it. "Never quote a frame rate without stating the exposure
# mode it was measured under" is a rule this project inherited the expensive
# way, and this line is what lets a gate obey it.
exposure_mode=$(grep -oP 'auto_exposure.*\(\K[^)]+' <<<"$after" | tail -n1)
dynamic_fps=$(awk '/exposure_dynamic_framerate/ {for (i=1;i<=NF;i++) if ($i ~ /^value=/) print substr($i,7)}' <<<"$after")

echo
echo "camera-reset device=${DEVICE}"
echo "camera-reset controls_written=${#sets[@]}"
echo "camera-reset refused=${refused}"
echo "camera-reset off_default=${off_default}"
echo "camera-reset exposure_mode=${exposure_mode:-unknown}"
echo "camera-reset exposure_dynamic_framerate=${dynamic_fps:-unknown}"

# exposure_dynamic_framerate is the assertion. Everything else here is a
# report; this one is the control that decides whether the next measurement is
# of a 20 fps camera or a 56 fps one, and a reset that silently failed to clear
# it would hand the gate a number it would then blame on the node.
if [[ ${dynamic_fps:-1} != 0 ]]; then
    echo "FAIL: exposure_dynamic_framerate is ${dynamic_fps:-unreadable}, expected 0"
    exit 1
fi
echo "PASS camera-reset"
