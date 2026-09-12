# `config/camera_info/`

Where a real calibration lands. One file per camera *and resolution*, named for
both: `c922_720p.yaml`.

`pimesh_camera`'s `camera_node` reads
`package://pimesh_bringup/config/camera_info/c922_720p.yaml` by default, so the
file being here is the whole of "the camera is calibrated" — there is no flag to
set, and that is deliberate (a `calibrated` parameter existed until P9 and let a
human assert the claim the startup warning exists to police).

The format is the standard `camera_info` YAML, which is exactly what
`camera_calibration`'s `cameracalibrator` writes as `ost.yaml`. It is copied
here verbatim rather than transcribed into a parameter file: fourteen numbers
retyped by hand is fourteen chances to be silently wrong.

Produce one with `bash tools/calibrate.sh` and check it with
`bash tools/gates/calibration.sh`. The resolution in `image_width`/`image_height`
must match the stream — a 720p calibration on a 1080p capture is wrong by a
constant factor in fx, fy, cx and cy at once, so `camera_node` refuses to start
on that mismatch rather than publishing it.

`c922_720p.yaml` is the C922 at 1280×720, fitted 2026-09-12 (P9,
[#9](https://github.com/bthek1/ros2_pi/issues/9)): fx=953.4, fy=957.6, cx=627.7,
cy=334.6, held-out reprojection 0.4955 px over 24 marker-confirmed frames. Its `D` is
near zero and **that is correct** — this camera measured as having essentially no lens
distortion in this mode, which is not what the phase expected; see
[hardware.md](../../../../docs/info/hardware.md#this-camera-has-essentially-no-distortion-at-720p).

One file per camera *and resolution*: these intrinsics do not carry to another mode,
and `camera_node` refuses to start rather than publish a calibration whose
`image_width`/`image_height` disagree with what it is capturing.
