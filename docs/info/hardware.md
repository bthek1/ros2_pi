# Hardware

All figures measured **2026-09-01** unless a different date is given. Re-measure
and update the date when anything changes — a stale spec here has already sent
this project's predecessor down a wrong path more than once.

## Dev box (here)

| | |
| --- | --- |
| Host | `proxmox-ml5`, a VM on a Proxmox host |
| OS | Ubuntu 26.04.1 LTS "resolute", x86_64 |
| Kernel | `7.0.0-30-generic` |
| CPU | AMD Ryzen 9 7900X, 16 vCPUs presented to the VM, 1 thread/core |
| RAM | 18 GB |
| GPU | **NVIDIA GeForce GTX 1660 SUPER**, 6144 MiB, compute capability **7.5**, driver **595.84** |
| ROS | **Lyrical**, `/opt/ros/lyrical` |
| LAN | `ens18` |
| Display | GNOME + Xwayland; the only host that can run `rviz2` |
| `sudo` | prompts for a password; the default `sudo` is `sudo-rs` |

### GPU notes that matter

- **Turing TU116.** Compute 7.5, but **no tensor cores** — this is the 1660, not
  an RTX part. fp16 gives bandwidth, not matrix throughput. Do not budget a
  TensorRT fp16 speedup you have not measured.
- 6 GB is comfortable for a ViT-S at 518² (well under 1 GB of activations) and
  would be tight for anything much larger.
- **Measured, via the predecessor's ONNX Runtime CUDA path:** Depth Anything V2
  Small at 518² runs **72–79 ms/frame**, with ~1.3 s of first-inference warm-up.
  The same model on CPU runs **280–305 ms/frame**. That 4× is the whole reason
  the GPU is in the design.
- **No CUDA toolkit is installed** — `nvcc` is absent and there is no
  `libcudart` in `/usr/lib/x86_64-linux-gnu`. The driver alone is enough for
  pip's `onnxruntime-gpu` (its wheels vendor the runtime) and **not enough for a
  C++ build**. See [setup.md](setup.md#gpu).

### Libraries present

| | Version | Notes |
| --- | --- | --- |
| OpenCV | 4.10.0 (apt) | **No CUDA module** — `cv::cuda::` will not link |
| PCL | 1.15.1 (`libpcl-dev` installed) | Plus `ros-lyrical-pcl-ros` |
| Open3D | **not installed** | Deliberate — see [pipeline.md](pipeline.md) |
| ONNX Runtime | **not installed for C++** | Python venv has 1.29.0 with CUDA + TensorRT providers |
| CUDA toolkit | **not installed** | Driver only |

## Raspberry Pi

| | |
| --- | --- |
| Model | **Raspberry Pi 5 Model B Rev 1.0** |
| Host | `raspberrypi`, `192.168.2.17`, reachable as `ssh pi` |
| OS | Ubuntu 24.04.4 LTS "noble", aarch64 |
| Kernel | `6.8.0-1060-raspi` |
| CPU / RAM | 4 cores / 8 GB (7937 MiB) |
| ROS | **Jazzy**, `/opt/ros/jazzy`, `ros-jazzy-ros-base` from apt |
| LAN | **`wlan0` — Wi-Fi.** `eth0` has no carrier; there is no cable |
| `sudo` | passwordless |

The Pi is on Wi-Fi, and that is a design input, not a detail: bandwidth is
shared and lossy, the link has died twice while the OS kept running, and a bad
network change leaves the machine needing a keyboard and a monitor. Treat
network changes on the Pi as higher-risk than they look.

## Camera

**Logitech C922 Pro Stream Webcam**, USB, on `usb-xhci-hcd.0-1`.

| | |
| --- | --- |
| Capture node | **`/dev/video0`** — `crw-rw---- root video` |
| `/dev/video1` | **Not a capture device.** It is the C922's UVC metadata node |
| Formats | `YUYV 4:2:2` and `MJPG`, both at 640×480 and 1280×720 |
| Permissions | The Pi's user is in `video`, so no `sudo` is needed |

The `/dev/video2x` nodes on the Pi belong to `pispbe`, the Pi 5's own image
signal processor. They are not this camera and there is no CSI camera attached —
`libcamera`/`rpicam` guidance does not apply.

### Capture behaviour (inherited from `piros2`, **re-verified here 2026-09-09**)

The two bullets below about frame rate are no longer inherited. `pimesh_camera`
was built and measured on 2026-09-09 with `bash tools/gates/capture.sh`, after
`bash tools/camera-reset.sh`, in Aperture Priority Mode with
`exposure_dynamic_framerate=0`: **59.3 fps measured at the Pi**, **44.3–58.6 fps
as received on the dev box** over five runs, **0 duplicate payloads** in any of
them, ~80 kB per frame. The spread is the Wi-Fi hop, not the camera. The 18–21
fps stock-settings figure has not been re-measured here and remains inherited.


- **Stock settings give 18–21 fps, not 30.** `exposure_dynamic_framerate=1`
  trades frame rate for exposure in indoor light, and the C922 powers on with it
  set despite the driver reporting the default as 0.
- **With the control cleared, 720p MJPG delivered 42–60 distinct frames/s**
  (measured 2026-08-04: 0 duplicate payloads in 634 messages). Budget every
  consumer for up to 60 fps.
- **V4L2 controls persist inside the camera** across processes and reboots. A
  manual exposure left behind by a benchmark makes every later session black.
  Reset the controls to a known baseline before diagnosing black frames or low
  frame rate as a software bug.
- Gain is never auto-adjusted on Linux; a dim room needs it raised by hand.
- **Never quote a frame rate without stating the exposure mode it was measured
  under.**

## Calibration target

**`docs/charuco_a4_7x9_25mm.pdf`, printed and mounted flat on a wall.** Hardware,
not documentation: it is a physical object whose dimensions every intrinsic this
project measures is scaled by, so it belongs here with the rest of the measured
specs. Generated by the predecessor's `piros2/tools/calib/make_calib_target.py`.

| | Nominal (as designed) | **Measured 2026-09-12** |
| --- | --- | --- |
| Scale bar on the sheet | 100 mm | **99 mm** (ruler) |
| Scale factor | 1.0000 | **0.9900** |
| Square | 25 mm | **24.75 mm** → `--square 0.02475` |
| ArUco marker | 18 mm | **17.82 mm** → `--marker 0.01782` |
| Squares | 7 × 9 | unchanged by scaling |
| Interior corners | 6 × 8 | unchanged by scaling |
| Dictionary | `DICT_4X4_250` (`4x4_250`) | unchanged |
| Markers on the board | 31 (ids 0–30) | 31 found in the PDF render |

**The print came out 1% small, and that 1% is not correctable in software.** The
sheet says "PRINT AT 100%" and it still scaled; this is the single most common way
calibrations come out subtly wrong, and nothing downstream can detect it — a 1%
error in `--square` is a 1% error in every distance the pipeline ever reports.
Passing the *nominal* 0.025 would have put that 1% into the reconstruction silently.
**Use the measured numbers above, not the ones in the filename.**

The nominal figures are confirmed exact, so the only error is the print scale: the
PDF rendered at 300 dpi measures 210.1 × 297.0 mm (A4), its 31 markers measure
**17.9977 mm** (0.01% off the labelled 18 mm), and the marker-to-square ratio is
0.7199 against a designed 18/25 = 0.72.

**A ratio measured off camera frames is not the way to check this.** The same ratio
taken from `bags/cam_2026_09_12` comes out at 0.654–0.674, about 7% low, and it is
unchanged by `CORNER_REFINE_SUBPIX`. The markers are only ~32 px across at 49 cm,
so their 5-px black border loses roughly a pixel per side to blur and JPEG, biasing
the marker inward while the chessboard corner pitch is unaffected. Measure the sheet
or the PDF, never the picture of it.

### Mounted on a wall it is rigid and **not flat** — measured 3 mm of bow

**Rigid and flat are different properties, and only the first is what a wall gives
you.** The phase's precondition says to mount the sheet rigidly so it cannot flex
while being waved about; taping it to a wall satisfies that completely and still
leaves the paper bowed, because paper taped at its edges bulges between them.

Measured 2026-09-12 from the 75 frames of `calib/c922_720p/frames/`, by solving a
per-corner out-of-plane offset jointly with the intrinsics across all views (the
plane's own `1, x, y` component projected out first, so board pose cannot be
absorbed into the shape):

| | planar model assumed | per-corner bow solved |
| --- | --- | --- |
| reprojection error | **1.0400 px** | **0.7732 px** |
| `k1` | **−0.0569** (pincushion) | **+0.0098** (barrel) |
| `fx` | 919.2 | 936.7 |
| `cx` | 592.1 | 624.3 |

**The sign of `k1` is the tell.** A C922 has barrel distortion, so `k1` must be
positive. Forcing a flat model onto a bowed board manufactures pincushion to absorb
the bulge, and every intrinsic shifts to accommodate it — `cx` by 32 px. The
recovered bow is **3.02 mm peak to peak**, 0.71 mm rms, in a vertical wave whose top
row stands ~1.9 mm proud of the middle.

**It is a real surface, not a fitting artefact.** Solved twice from disjoint halves
of the frame set, the two surfaces correlate at **+0.994** and differ by 0.08 mm rms.

So: mount the sheet on foam board, MDF or a clipboard, as the phase says, and then
put *that* on the wall. A print taped flat to a wall is not a calibration target.

### A second, smaller error: the markers corrupt `cornerSubPix`

The ArUco marker is 18 mm inside a 24.75 mm square, so its black border sits
3.375 mm from each chessboard corner — about **6.7 px** at the observed 49 px square
pitch, which is *inside* the default 11×11 sub-pixel refinement window. The marker
edge then pulls the saddle-point fit. Measured over the same 75 frames:

| `cornerSubPix` window | reprojection error |
| --- | --- |
| 11×11 (OpenCV default) | 1.0400 px |
| 9×9 | 1.0079 px |
| **7×7** | **0.8707 px** |
| 5×5 | 0.8969 px |
| 4×4 | 1.0290 px |
| 3×3 | 1.1705 px |

7×7 is the optimum here and worth ~17%; below that, too few pixels remain to fit.
It does not change the sign of `k1`, so it is a real but secondary effect — the bow
is the dominant one. Combining both (7×7 *and* a solved bow) reaches **0.6684 px**,
which is the best estimate of what a genuinely flat print of this sheet would give
and is still above the gate's 0.5 px budget. Expect to need the flat mount *and*
good frames.

### This camera has essentially no distortion at 720p

Measured 2026-09-12, and it contradicts what this project assumed when P9 was
written. Three independent lines of evidence:

| test | result | what distortion would give |
| --- | --- | --- |
| correlation of board curvature with distance from image centre, 243 frames | **−0.160** | strongly positive |
| median raw board deviation by reach band (0.35 / 0.50 / 0.65 / 0.80 / 1.1) | 0.93, 0.74, 0.65, 0.54, 0.65 px | rising steeply |
| real straight edges, 430–473 px long at 0.62–0.73 reach | **0.93–1.42 px** bow | several × more at `k1=+0.08` |
| `k1` from every real calibration | **\|k1\| < 0.02**, sign flips with frame count | a stable ≈ +0.05…+0.1 |

Most likely Logitech corrects distortion in firmware for this mode. **Do not assume
it holds at another resolution** — a cropped or uncorrected mode may behave
completely differently, and this is a per-mode measurement.

Two consequences for the gate, both applied:

- **Never assert `k1 > 0`.** It fails a *correct* calibration of this camera. It was
  added as a flat-board detector and did work as one against a 3 mm bow, but it
  cannot tell "the board is bowed" from "this lens is straight", and only one of
  those is a fault. The flatness check that survives is the reprojection error, which
  a bowed board inflates whatever the lens does.
- **Straightness cannot be "strictly better than the placeholder".** With no
  distortion to remove, the placeholder is already almost right and the two figures
  are equal to within noise. The gate asserts *not worse* plus the absolute budget,
  and prints the ratio so a camera that does have distortion would still show it.

### Acquire the frames from a bag, not live

`bash tools/calibrate.sh record` then `select` is the better of the two paths, and
the live `grab` is kept only for a quick look. Recording separates the physical job
from the judgement: at the wall you only have to move the camera slowly, and the
choosing happens afterwards with every candidate on the table at once instead of one
at a time as they arrive. The bag is also re-selectable — a different count, a
tighter obliquity bound — without another visit.

`select` confirms every frame against the markers, drops frames below half the bag's
median sharpness, rejects views past 45° oblique, then picks for **coverage first and
pose diversity second** in `cameracalibrator`'s own (x, y, size, skew) space. It
prints what it rejected and warns when the bag itself was inadequate — run against
`bags/cam_2026_09_12` it reports `coverage=0.446`, `obliquity spread=0.1 deg`,
`distance spread=0 cm`, which is the correct verdict on a bag recorded from a tripod.

### The detector this project uses on it

The gate reads this ChArUco board with the **plain `findChessboardCorners` at
6 × 8**, not with `cv2.aruco`, because `cv2.aruco`'s API differs between the Pi's
OpenCV 4.6 (no `CharucoDetector`) and this box's 4.10, and an instrument that
differs per machine is the ABI problem in a new costume.

**That it works at all is verified rather than assumed**, since the markers sitting
in the white squares might be expected to break the detector's quad topology. On a
frame from `bags/cam_2026_09_12` (2026-09-12), all **42** corners `cv2.aruco`
interpolated agree with the `findChessboardCorners` grid *at the same (row, col)
index*, median 0.73 px apart and 1.48 px at worst — so the grid is the board's true
interior corners and not some coincidental lattice. 41 of 41 sampled frames detected.

One oddity worth recording so it is not mistaken for a fault: the detector **fails
on the 300 dpi flat render** of the same sheet and succeeds at every
camera-realistic size (measured: False at 2481 × 3508, True at 1240 × 1754 and
below). It is a resolution effect in the detector, not a property of the board.

**But its grid must be confirmed against the markers, because it can be a square
out.** Measured 2026-09-12 over the 35 frames of a real grab set: `frame-13.jpg`
produced a genuine, internally consistent 6 × 8 corner lattice that disagreed with the
ChArUco ids by **45.8 px median, 50.0 px worst** — one square pitch — while being
sharp, 148 kB, and only 18° oblique. Nothing about the detection announced it, and its
corners were matched to object points one square out, so it pulled the whole solve:

| frame set | reprojection | straightness vs control |
| --- | --- | --- |
| all 35 as grabbed | 0.7684 px | 4.163 px vs 4.174 px (no improvement) |
| **20 marker-confirmed** | **0.4548 px** | 1.093 px vs 1.179 px |

Held-out halves of the confirmed set gave 0.4643 and 0.4701 px. So confirmation is
required, not advisory, and `confirm_grid()` does it in both the grabber and the gate.

The same single check rejects three failure modes, which is why it is one check:
misregistration, motion blur (markers will not decode), and obliquity past ~50° (same
reason). That last is no loss — those frames were independently the least accurate:

| board obliquity | frames | mean reprojection | mean straightness |
| --- | --- | --- | --- |
| 18° (the misregistered one) | 1 | 1.789 px | 4.163 px |
| **20–35°** | **16** | **0.429 px** | **0.579 px** |
| 35–50° | 8 | 0.686 px | 0.974 px |
| 50–61° | 10 | 0.933 px | 1.319 px |

**20–35° is the band to shoot in.** Beyond 50° the board is foreshortened about 2:1
(measured side-length ratio 0.47), which halves the effective resolution along one
axis and the corner fit with it.

**What this choice costs, now measured rather than anticipated.**
`findChessboardCorners` needs the whole board in shot, so a frame with the board
clipped at a frame edge is skipped entirely — and that is exactly where the coverage
floor pushes it. On seven frames with the board running off the right edge
(2026-09-12):

| detector | corners recovered |
| --- | --- |
| `findChessboardCorners` (6×8) | **0 on all seven** |
| `interpolateCornersCharuco` | **23–30 of 48 (48–62%) on all seven** |

So the two requirements are in direct tension with this detector: reaching a frame
corner tends to put part of the board outside the frame, and then nothing is
detected at all. It is also what produced a live session reporting "board seen in 0
frames of 1086" with the board plainly on the wall.

**The fix is to make ChArUco the primary detector, not merely the confirmer** —
`interpolateCornersCharuco` is the one aruco entry point present on both 4.6 and
4.10, and its corners are identified, which additionally removes misregistration, the
180° ambiguity and the transposed-`--size` hazard by construction rather than by
check. It needs sparse handling throughout: line fitting over the corners that are
present, per-frame object points from the ids, and coverage over sparse points.
Until then, **keep the whole board inside the frame**, which caps coverage at about
0.86 on this board at 55 cm.

## Network

| | |
| --- | --- |
| Subnet | `192.168.2.0/24` |
| Dev box | on `ens18` |
| Pi | `192.168.2.17` on `wlan0` |
| `ROS_DOMAIN_ID` | **42** on both machines (0 is shared with everything else on the LAN) |
| RMW | `rmw_cyclonedds_cpp` on both |
| DDS interface | pinned per host via `CYCLONEDDS_URI` |

The dev box has interfaces DDS must **not** bind to: Docker bridges
(`172.17`–`172.19`) and a Tailscale interface. Left unpinned, DDS will pick one
and advertise an address the Pi cannot route to — and a VPN interface is the
nastier case, because it looks routable and is not.

**The two machines run different ROS distros** (Lyrical here, Jazzy there).
Interop was measured working in the predecessor project on 2026-08-31 — topics,
`camera_info` byte-identical, and `tf_static` all crossed the LAN. There is no
ABI compatibility across distros, so **shared libraries do not cross**; every
package builds from source on the machine that runs it.
