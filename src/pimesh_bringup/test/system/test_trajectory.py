"""P11's trajectory gate as a launch_testing test: #15's P3 pilot.

    launch_test src/pimesh_bringup/test/system/test_trajectory.py

The same claim as `tools/gates/trajectory.sh`, made by the mechanism ROS 2 built
for a system test: `launch_testing` brings the launch up, runs *active* tests
while it is running, shuts it down, and runs *post-shutdown* tests over what it
left behind. Three launches, by `launch_testing.parametrize`:

- `sixdof`: TUM fr1/desk through the real container, `odom_probe` writing the TUM
  trajectory. Asserts the Sim(3) ATE under the ceiling, the replay real time and
  complete, enough poses, the poses associated with the truth, the solve's
  residual, and the intrinsics **off the wire** equal to the dataset's.
- `rotation_only`: the control. Translation identically zero, and the ceiling
  must not admit it — Umeyama refuses to align a trajectory with no translation,
  which is itself the strongest form of "does not reach the ceiling".
- `wrongcal`: control 0. `dataset_node` alone, handed the C922's 1280x720
  calibration over 640x480 frames, must refuse to start and say why.

**Not registered with `colcon test`, on purpose.** It needs the dataset, the
model, the GPU and evo, takes minutes, and exists only on the dev box —
`gates/test.sh` asserts the same suites at both ends, and evo is not on the Pi.

**evo through its saved results, not its stdout.** evo lives in a uv venv the
system interpreter cannot import, so it is run as a process with
`--save_results`, and the numbers are read from the zip it writes: `stats.json`
for the RMSE, `timestamps.npy` for how many poses it associated, and the Sim(3)
matrix for the fitted scale. The bash gate scrapes the text evo prints; a
reworded line there reads as "no ATE", here it is a missing key.

**What it does not do that the bash gate does: nothing.** What it does that the
bash gate does not: it reads `depth_scale` from the YAML for the implied-scale
line, where `trajectory.sh` multiplied by a 10.0 typed before P12 pinned 4.6002.
"""

import json
import math
import os
import re
import subprocess
import tempfile
import time
import unittest
import zipfile
from pathlib import Path

import launch_testing
import launch_testing.asserts
import numpy as np
import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# The bash gate's thresholds, unchanged: parity is the point of the pilot.
MAX_ATE_M = float(os.environ.get('PIMESH_MAX_ATE_M', '0.60'))
RPE_WINDOW_S = 1.0
MIN_POSES = 250
MIN_ASSOCIATED_PCT = 99.0
MIN_POSED_PCT = 70.0
MAX_REPROJ_PX = 2.0

SHARE = get_package_share_directory('pimesh_bringup')
WS = Path(os.environ.get('PIMESH_WS') or Path(SHARE).resolve().parents[3])
SEQ = Path(subprocess.run(['bash', str(WS / 'tools' / 'fetch-dataset.sh'), '--print-path'],
                          capture_output=True, text=True).stdout.strip())
GROUND_TRUTH = SEQ / 'groundtruth.txt'
DATASET_CAL = WS / 'src/pimesh_bringup/config/camera_info/tum_freiburg1.yaml'
CAMERA_CAL = 'package://pimesh_bringup/config/camera_info/c922_720p.yaml'
EVO_APE = os.environ.get('EVO_APE', str(Path.home() / '.local/bin/evo_ape'))
EVO_RPE = os.environ.get('EVO_RPE', str(Path.home() / '.local/bin/evo_rpe'))
WORK = Path(tempfile.mkdtemp(prefix='pimesh-trajectory-'))


def _sequence():
    stamps = [float(line.split()[0]) for line in open(SEQ / 'rgb.txt')
              if line.strip() and not line.startswith('#')]
    return len(stamps), stamps[-1] - stamps[0]


SEQ_FRAMES, SEQ_SECONDS = _sequence()
MEASURE_S = int(SEQ_SECONDS + 8)


def stationary_ate() -> float:
    """What an estimate that never moves scores: the truth's RMS spread about its centroid.

    Derived from groundtruth.txt, never typed — the ceiling has to sit below it.
    """
    rows = np.array([[float(v) for v in line.split()[1:4]]
                     for line in open(GROUND_TRUTH) if not line.startswith('#')])
    return float(np.sqrt(((rows - rows.mean(axis=0)) ** 2).sum(axis=1).mean()))


def evo(tool: str, traj: Path, *args) -> dict:
    """Run evo, read its saved results: rmse, associated pairs, fitted scale."""
    out = WORK / f'{traj.stem}-{Path(tool).name}-{"-".join(args) or "x"}.zip'
    out.unlink(missing_ok=True)
    proc = subprocess.run([tool, 'tum', str(GROUND_TRUTH), str(traj), '-a', *args,
                           '--save_results', str(out), '--no_warnings'],
                          capture_output=True, text=True)
    if not out.is_file():
        return {'rmse': None, 'why': (proc.stdout + proc.stderr)[-400:]}
    with zipfile.ZipFile(out) as z:
        stats = json.loads(z.read('stats.json'))
        names = z.namelist()
        pairs = len(np.load(z.open('timestamps.npy'))) if 'timestamps.npy' in names else 0
        scale = 1.0
        if 'alignment_transformation_sim3.npy' in names:
            m = np.load(z.open('alignment_transformation_sim3.npy'))
            scale = float(np.cbrt(np.linalg.det(m[:3, :3])))
    return {'rmse': stats['rmse'], 'pairs': pairs, 'scale': scale}


def last_kv(text: str, prefix: str, key: str, need_rate=False):
    """The last value of `key` on lines starting with `prefix` — the gate's awk, in Python.

    `need_rate` skips the idle tail windows (rate < 5 Hz), as trajectory.sh does:
    the last stats window of a run covers the seconds after the clip ended.
    """
    value = None
    for line in text.splitlines():
        i = line.find(prefix)
        if i < 0:
            continue
        kv = dict(t.split('=', 1) for t in line[i:].split() if '=' in t)
        if need_rate and float(kv.get('rate', '0').rstrip('Hz') or 0) < 5:
            continue
        if key in kv:
            value = kv[key]
    return value


@pytest.mark.launch_test
@launch_testing.parametrize('case', ['sixdof', 'rotation_only', 'wrongcal'])
def generate_test_description(case):
    if case == 'wrongcal':
        node = Node(package='pimesh_dataset', executable='dataset_node', output='screen',
                    parameters=[{'dataset_dir': str(SEQ), 'max_frames': 2,
                                 'camera_info_url': CAMERA_CAL}])
        return LaunchDescription([node, launch_testing.actions.ReadyToTest()]), {
            'case': case, 'proc': node}
    traj = WORK / f'{case}.tum'
    traj.unlink(missing_ok=True)
    pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(SHARE, 'launch', 'pimesh.launch.py')),
        launch_arguments={
            'source': 'dataset_node', 'dataset_dir': str(SEQ), 'odom_regime': case,
            'probe': 'odom_probe', 'probe_duration_s': f'{MEASURE_S + 4:.1f}',
            'trajectory_path': str(traj),
        }.items())
    return LaunchDescription([
        pipeline,
        # ReadyToTest after a beat, so the active test's clock starts with the
        # container rather than before it.
        TimerAction(period=1.0, actions=[launch_testing.actions.ReadyToTest()]),
    ]), {'case': case, 'proc': None, 'traj': traj}


class TestWhileRunning(unittest.TestCase):

    def test_replay_completes_and_intrinsics_reach_the_wire(self, proc_info, proc_output,
                                                            case, proc):
        if case == 'wrongcal':
            # Control 0 does not come up; its assertion is on how it ended — so wait
            # for it to end **by itself**. The first version waited for the refusal's
            # log line and returned, launch_testing then shut the node down, and the
            # post-shutdown check read SIGINT's -2 as "refused": a node that never
            # refused anything would have passed (found 2026-10-05, three runs).
            proc_info.assertWaitForShutdown(process=proc, timeout=20)
            return
        proc_output.assertWaitFor(re.compile(r'dataset .*replaying at'), timeout=60)
        if case == 'sixdof':
            served = _served_camera_info()
            (WORK / 'served_k.json').write_text(json.dumps(served))
        proc_output.assertWaitFor('dataset finished', timeout=MEASURE_S + 30)
        # odom_probe prints its summary when its window closes, after the clip.
        proc_output.assertWaitFor('odom_probe result', timeout=60)
        proc_output.assertWaitFor('odom_probe trajectory', timeout=30)


def _served_camera_info():
    """K and width as published on /camera_info: off the wire, not off a log line."""
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo
    rclpy.init()
    try:
        node = rclpy.create_node('trajectory_test_camera_info')
        got = []
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        node.create_subscription(CameraInfo, '/camera_info', got.append, qos)
        deadline = time.monotonic() + 20
        while not got and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        node.destroy_node()
        return {'k': list(got[0].k), 'width': got[0].width} if got else None
    finally:
        rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestAfterShutdown(unittest.TestCase):

    # One post-shutdown test that branches on the case, not one per claim with the
    # others skipped: a skipped test reports a run that measured nothing, and this
    # project's gates count skips as failures for that reason.
    def test_trajectory(self, proc_info, proc_output, case, proc):
        if case == 'wrongcal':
            rc = proc_info[proc].returncode
            # > 0, not != 0: a negative return code is a signal, i.e. something
            # else ended it, and that is not a refusal.
            self.assertGreater(rc, 0, "dataset_node did not refuse the C922's calibration "
                                      f'over 640x480 frames by itself (exit {rc})')
            text = ''.join(o.text.decode('utf-8', errors='replace') for o in proc_output[proc])
            self.assertRegex(text, r'1280|720|width|height|resolution',
                             'refused, but not for the resolution')
            print(f'\ncontrol 0: dataset_node refused the C922 calibration, exit {rc}')
            return
        text = ''.join(o.text.decode(errors='replace') for o in proc_output)
        traj = WORK / f'{case}.tum'
        poses = sum(1 for line in open(traj) if not line.startswith('#')) \
            if traj.is_file() else 0
        path_m = float(last_kv(text, 'odom_probe result', 'path_m') or 'nan')
        sim3 = evo(EVO_APE, traj, '-s') if poses else {'rmse': None}
        stationary = stationary_ate()
        self.assertLess(MAX_ATE_M, 0.95 * stationary,
                        f'the {MAX_ATE_M} m ceiling is not below the {stationary:.4f} m a '
                        'trajectory that never moves would score')

        if case == 'rotation_only':
            self.assertEqual(path_m, 0.0, 'the control published translation')
            if sim3['rmse'] is not None:
                self.assertGreater(sim3['rmse'], MAX_ATE_M,
                                   'the ceiling admits a trajectory that never moves')
            print(f'\ncontrol rotation_only: path {path_m} m over {poses} poses, '
                  f'ATE {sim3["rmse"]} (stationary {stationary:.4f} m)')
            return

        se3 = evo(EVO_APE, traj)
        rate = float(last_kv(text, 'odom_probe result', 'rate') or 0)
        delta = max(2, int(rate * RPE_WINDOW_S + 0.5))
        rpe = evo(EVO_RPE, traj, '-s', '--delta', str(delta), '--delta_unit', 'f',
                  '--all_pairs')
        finished = re.search(r'dataset finished: (\d+) frames published, (\d+) behind', text)
        skipped = re.findall(r'odom_probe trajectory poses=\d+ skipped=(\d+)', text)
        reproj = float(last_kv(text, 'stats regime=', 'reproj_px', True) or 0)
        ok = float(last_kv(text, 'stats regime=', 'shift_ok', True) or 0)
        held = float(last_kv(text, 'stats regime=', 'shift_held', True) or 0)
        posed_pct = 100 * ok / (ok + held) if ok + held else 0.0
        with open(WS / 'src/pimesh_bringup/config/pimesh.yaml') as fh:
            depth_scale = yaml.safe_load(fh)['/**/depth_node']['ros__parameters']['depth_scale']

        print(f'\n== test_trajectory (launch_testing pilot) ==\n'
              f'ATE RMSE, SE(3) aligned (m)   {se3["rmse"]}\n'
              f'ATE RMSE, Sim(3) aligned (m)  {sim3["rmse"]}\n'
              f'RPE RMSE over {RPE_WINDOW_S}s (m)       {rpe["rmse"]}  (window {delta} frames)\n'
              f'fitted scale s                {sim3.get("scale")}\n'
              f'poses {poses}, associated {sim3.get("pairs")}, posed {posed_pct:.1f}%, '
              f'reprojection {reproj} px\n'
              f'depth_scale implied           {depth_scale * sim3.get("scale", math.nan):.4f} '
              f'(config/pimesh.yaml says {depth_scale})')

        self.assertIsNotNone(finished, 'dataset_node never reported finishing')
        self.assertEqual(int(finished.group(1)), SEQ_FRAMES, 'replay incomplete')
        self.assertEqual(int(finished.group(2)), 0, 'replay was not real time')
        self.assertTrue(skipped and int(skipped[-1]) == 0,
                        'poses left out of the trajectory for want of the optical frame')
        self.assertGreaterEqual(poses, MIN_POSES)
        self.assertIsNotNone(sim3['rmse'], f'evo could not align it: {sim3.get("why")}')
        self.assertGreaterEqual(100 * sim3['pairs'] / poses, MIN_ASSOCIATED_PCT)
        self.assertTrue(0.01 <= reproj <= MAX_REPROJ_PX, f'reprojection {reproj} px')
        self.assertGreaterEqual(posed_pct, MIN_POSED_PCT)
        self.assertLessEqual(sim3['rmse'], MAX_ATE_M)

        served = json.loads((WORK / 'served_k.json').read_text())
        self.assertIsNotNone(served, 'nothing was served on /camera_info — could not check, '
                                     'which is not the same as passing')
        with open(DATASET_CAL) as fh:
            want = yaml.safe_load(fh)['camera_matrix']['data']
        for i, (a, b) in enumerate(zip(served['k'], want)):
            self.assertAlmostEqual(a, b, places=6, msg=f'K[{i}] served {a}, file says {b}')
