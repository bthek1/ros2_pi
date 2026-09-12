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

Nothing is committed here yet: until the checkerboard has been run, the absence
of the file is the honest state, and `camera_node` says so on every startup.
