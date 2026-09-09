#!/usr/bin/env bash
#
# Watch the Pi's camera, live, with the frame tree next to it.
#
# **A viewer, not evidence.** tools/gates/capture.sh is what passes or fails P1.
# This is here so a person can confirm with their own eyes that the thing is
# alive before trusting a number about it — and so that "the mesh looks like the
# room", later, has somewhere to be looked at.
#
# It starts the camera on the Pi and RViz here, and it tears both down on Ctrl-C
# or a closed window. The teardown is the fiddly part and it is all in
# just-lib.sh: arm_cleanup installs a handler on EXIT and on INT/TERM/HUP that
# pkills the node patterns on both machines, and run_for keeps rviz2 in this
# shell's process group so a terminal's Ctrl-C reaches it at all.

source "$(dirname "${BASH_SOURCE[0]}")/just-lib.sh" --overlay

SECONDS_LIMIT=${1:-600}
RVIZ_CONFIG="$(ros2 pkg prefix pimesh_bringup)/share/pimesh_bringup/rviz/camera.rviz"

arm_cleanup

cat <<'CHECKLIST'
== view-camera ==

What you should see, and what each part of it tells you:

  Image    live video from the C922, at roughly the rate gates/capture.sh
           printed. If this is grey, the LAN hop or the QoS match is broken,
           not the camera — check `ros2 topic hz /image_raw/compressed` first.
  TF       three frames: base_link -> camera_link -> camera_optical_frame, with
           the optical frame rotated into z-forward (its blue axis points the
           way the camera looks, its red axis to the image's right).
  Fixed Frame is base_link.

This window is a *second* RELIABLE subscriber on the only topic that crosses
Wi-Fi. That is exactly what the architecture forbids at scale and it is
tolerable here only because a compressed frame is ~86 kB. Close it before
measuring anything.

CHECKLIST

[[ -r $RVIZ_CONFIG ]] || { echo "no RViz config at $RVIZ_CONFIG — build first"; exit 1; }

# The camera, on the Pi, bounded. pi_run_for puts `timeout` inside the login
# shell so the limit reaches the node rather than the shell wrapping it.
pi_run_for "$SECONDS_LIMIT" "ros2 run pimesh_camera camera_node" &

# The frames, here. Static transforms are latched, so this only has to be up
# before RViz asks — but it stays up because a transient-local publisher that
# exits takes its data with it.
ros2 launch pimesh_bringup pimesh.launch.py >/dev/null 2>&1 &

sleep 3

# The session is Wayland; rviz2 renders through GLX and needs the xcb platform
# plugin. Hardware GL 4.6 on driver 595.84 as of 2026-08-31, so the old
# software-GL workaround (LIBGL_ALWAYS_SOFTWARE) is obsolete and would only make
# this slow.
export QT_QPA_PLATFORM=xcb

# Backgrounded, and waited on. This is the second of the two shapes
# just-lib.sh's run_for describes, and for this recipe it is the only correct
# one — the difference is worth stating, because the foreground form is what
# every other watchable recipe here uses and it is wrong precisely once.
#
# Bash will not run a trap while a foreground child is running; it defers the
# handler until that child returns. For `ros2 launch` that is harmless, because
# a launcher signalled in the same process group shuts itself down and returns
# promptly. **rviz2 does not.** Signalled during its own startup — after the
# process exists, before its Qt event loop is running — it records the shutdown
# request and never acts on it, and it is then immortal: the script sits in
# `wait` on a child that will not exit, its EXIT/INT trap never fires, and the
# camera_node on the Pi is never told to stop. That node holds /dev/video0
# exclusively, so the next session dies with "Device or resource busy" and the
# leak outlives the thing that caused it. Measured 2026-09-09: rviz2 alive 30 s
# after a group SIGINT, with the Pi's camera still streaming.
#
# Backgrounding it inverts that. The signal arrives, bash is idle in `wait`, and
# the trap runs *immediately* — cleanup_both pkills the viewer by pattern here
# and the camera over SSH, whatever state rviz2 had got itself into. Closing the
# window still ends the session, because `wait` returns when the child does.
run_for "$SECONDS_LIMIT" rviz2 -d "$RVIZ_CONFIG" &
wait $! || true
