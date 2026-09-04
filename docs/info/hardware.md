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
| Toolchain | g++ **13.3.0**, cmake **3.28.3** — measured 2026-09-02. g++ 13 is why the Pi's packages are C++17 |
| LAN | **`wlan0` — Wi-Fi.** `eth0` has no carrier; there is no cable |
| `sudo` | passwordless |
| Configured by | **Ansible** — [ansible.md](ansible.md). Nothing is installed on it by hand |

The Pi is on Wi-Fi, and that is a design input, not a detail: bandwidth is
shared and lossy, the link has died twice while the OS kept running, and a bad
network change leaves the machine needing a keyboard and a monitor. Treat
network changes on the Pi as higher-risk than they look. It is also the second
reason the Pi's configuration is a playbook rather than a shell history: the
recovery from a bad network change is a reflash, and a reflash loses everything
that was not written down as a role.

## Camera

**Logitech C922 Pro Stream Webcam**, USB, on `usb-xhci-hcd.0-1`.

| | |
| --- | --- |
| Capture node | **`/dev/video0`** — `crw-rw---- root video`, reached through the by-id symlink |
| USB link | **480M (USB 2.0)**, `Bus 002 … xhci-hcd`. See the rate note below |
| Stable path | `/dev/v4l/by-id/usb-046d_C922_Pro_Stream_Webcam_5461327F-video-index0` — survives replugs; prefer it over `/dev/video0` in config |
| `/dev/video1` | **Not a capture device.** It is the C922's UVC metadata node (`…-video-index1`) |
| Formats | `YUYV 4:2:2` and `MJPG`, both at 640×480 and 1280×720 |
| Permissions | The Pi's user is in `video`, so no `sudo` is needed |

The `/dev/video2x` nodes on the Pi belong to `pispbe`, the Pi 5's own image
signal processor. They are not this camera and there is no CSI camera attached —
`libcamera`/`rpicam` guidance does not apply.

### Capture behaviour

**Measured here 2026-09-02, via raw `v4l2-ctl` with no ROS in the loop:**

```
v4l2-ctl -d <by-id> --set-fmt-video=width=1280,height=720,pixelformat=MJPG \
         --set-parm=60 --stream-mmap --stream-count=200 --stream-to=/dev/null
→ 29.7 fps in one set of runs, 58.8 fps in another, same day, same command
```

**The rate is not a property of the camera — it tracks the auto-exposure
time.** Both figures came from the same device, the same link and the same
control baseline within an hour of each other; the only difference was the
light in the room and how long auto-exposure had had to converge. That is the
same behaviour the predecessor recorded (18–21 fps stock, 42–60 fps after
clearing the persistent controls, 2026-08-04), and it is why **no frame-rate
figure in this project means anything without the exposure conditions beside
it**.

Consequences that follow from this, and they are the practical ones:

- **A gate must measure the hardware's rate in the same run** it judges a node
  against, never compare against a number written down on another day.
  `just gate-capture` does exactly that: raw `v4l2-ctl` first, then the node,
  and the assertion is on the *ratio*.
- **Budget consumers for up to 60 fps** — the fast case is real.
- The driver reports `Frames per second: 60.000` after `VIDIOC_S_PARM`
  regardless, because that is the request being echoed back. **`v4l2-ctl`'s
  closing line `Frame rate set to 60.000 fps` is not a measurement either**;
  the measured number is on the streaming progress lines.

**The camera is on a 480M (USB 2.0) link** — `lsusb -t` shows
`Bus 002 … xhci-hcd/2p, 480M` with the C922 beneath it. 720p60 MJPEG fits in
that budget (~9 MB/s of ~24 MB/s après overhead), which is consistent with
58.8 fps being achievable; it is recorded here because it bounds anything
faster or larger.

Still true, and still inherited:

- **Stock settings give 18–21 fps, not 30.** `exposure_dynamic_framerate=1`
  trades frame rate for exposure in indoor light, and the C922 powers on with it
  set despite the driver reporting the default as 0.
- **V4L2 controls persist inside the camera** across processes and reboots. A
  manual exposure left behind by a benchmark makes every later session black.
  Reset the controls to a known baseline before diagnosing black frames or low
  frame rate as a software bug.
- Gain is never auto-adjusted on Linux; a dim room needs it raised by hand.
- **Never quote a frame rate without stating the exposure mode it was measured
  under.**

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
