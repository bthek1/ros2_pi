"""Record a clip from the Pi's camera into bags/<name>: #15's P2.

the deleted `just record` did this as a script whose steps were separated by sleeps
and polls: reset the camera, sleep 5, start the camera, poll `ros2 topic list`
until the topic appeared, record under a timer, kill everything, check for
metadata.yaml. Here each step starts on the **exit event** of the one before it,
which is what `launch`'s event handlers are for:

    camera_reset  --exit 0-->  camera_node (Pi) + first_frame
    first_frame   --exit---->  recorder, bounded to `seconds`
    recorder      --exit---->  check metadata.yaml, print the summary, Shutdown

**The recorder starts on the first frame, not on a guess.** `first_frame` is
`ros2 topic echo --once` on the camera's topic; its exit is "a frame has
arrived", so the clip's seconds are seconds of room and not of camera startup.

**The recorder ends itself.** `ros2 bag record` has no total-duration option
(`-d` only splits files), so it runs under `timeout -s INT <seconds>`: SIGINT is
what makes it write metadata.yaml, and the check that follows is in the event
chain rather than racing launch's own shutdown. A Ctrl-C reaches it too —
this box's uutils `timeout` forwards SIGINT to its child in both modes, measured
2026-10-05.

**Two refusals, both from the script it replaces**, and both before anything
runs or after everything has: a bag that already exists (a reference clip is
recorded once — delete it deliberately), and a recording that has no
metadata.yaml at the end, which is a recorder killed before it finished.

Run it through `tools/session.sh record --pi -- …` (`just record`), whose
`kill_pi` is what ends the camera on the Pi.
"""

import hashlib
import os
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo,
                            OpaqueFunction, RegisterEventHandler, Shutdown,
                            TimerAction)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration

TOPICS = ['/image_raw/compressed', '/camera_info']


def _workspace() -> Path:
    from ament_index_python.packages import get_package_share_directory
    env = os.environ.get('PIMESH_WS')
    if env:
        return Path(env)
    return Path(get_package_share_directory('pimesh_bringup')).resolve().parents[3]


def summarise(bag: Path) -> list:
    """The lines printed after a recording; raises if there is nothing to summarise.

    A function of the directory alone so test_views can call it on a fixture.
    """
    meta = bag / 'metadata.yaml'
    if not meta.is_file():
        raise RuntimeError(
            f'no metadata.yaml in {bag} — the recorder was killed before it finished')
    with open(meta) as fh:
        info = yaml.safe_load(fh)['rosbag2_bagfile_information']
    seconds = info['duration']['nanoseconds'] / 1e9
    lines = [f'== bags/{bag.name} ==']
    for entry in info['topics_with_message_count']:
        name, count = entry['topic_metadata']['name'], entry['message_count']
        lines.append(f'  {name:<28} {count} messages')
        if name.endswith('image_raw/compressed') and seconds > 0:
            lines.append(f"  {'image rate':<28} {count / seconds:.1f} Hz")
    lines.append(f"  {'duration':<28} {seconds:.2f}s")
    for mcap in sorted(bag.glob('*.mcap')):
        digest = hashlib.sha256(mcap.read_bytes()).hexdigest()
        lines.append(f"  {'sha256':<28} {digest}")
    lines.append(f'Look at it before trusting it:  just replay {bag.name}')
    return lines


def _setup(context):
    ws = _workspace()
    name = LaunchConfiguration('name').perform(context)
    seconds = int(float(LaunchConfiguration('seconds').perform(context)))
    if not name or '/' in name:
        raise RuntimeError(f"name:='{name}' must be a directory name under bags/")
    bag = ws / 'bags' / name
    if bag.exists():
        raise RuntimeError(
            f'bags/{name} already exists. A reference clip is recorded once — '
            'delete it deliberately if you really mean to replace it.')

    reset = ExecuteProcess(cmd=['bash', str(ws / 'tools' / 'calib' / 'camera-reset.sh')],
                           name='camera_reset', output='screen')
    camera = ExecuteProcess(
        cmd=['bash', '-c',
             # The explicit "" matters: a sourced file sees the caller's
                 # positional parameters, and just-lib.sh would read $1 — the
                 # workspace path — as an option and exit (found by #15's P1).
                 'source "$1/tools/lib/just-lib.sh" "" && '
             # `& wait`: see view.launch.py — bash defers SIGINT while a
             # foreground child runs, so launch's SIGINT waited out 5 s.
             'pi_run_for "$2" "ros2 run pimesh_camera camera_node" & wait $!',
             'pi_camera', str(ws), str(seconds + 60)],
        name='pi_camera', output='log')
    first_frame = ExecuteProcess(
        # The type is given so echo *waits* for a publisher: without it, echo
        # exits 1 at once on a topic that does not exist yet, which is every
        # topic for the seconds before the Pi's camera is up (found on the
        # first run, 2026-10-05).
        cmd=['ros2', 'topic', 'echo', '--once', '--field', 'header.stamp', TOPICS[0],
             'sensor_msgs/msg/CompressedImage'],
        name='first_frame', output='log')
    recorder = ExecuteProcess(
        # --preserve-status: report the recorder's own exit (0 after a clean
        # SIGINT) rather than timeout's 124, which launch logs as a death.
        cmd=['timeout', '--preserve-status', '-s', 'INT', str(seconds),
             'ros2', 'bag', 'record', '-s', 'mcap', '-o', str(bag), '--topics', *TOPICS],
        name='recorder', output='log')

    def after_reset(event, _context):
        if event.returncode != 0:
            # camera-reset.sh exits non-zero when the one control that matters
            # did not stick; a clip recorded under it cannot be un-recorded.
            return [LogInfo(msg='camera-reset.sh failed: not recording'),
                    Shutdown(reason='camera reset failed')]
        return [camera, first_frame]

    def after_first_frame(event, _context):
        if event.returncode != 0:
            # Recording anyway would print "first frame arrived" over a camera
            # that never published — the first run did exactly that.
            return [LogInfo(msg=f'no frame on {TOPICS[0]} (exit {event.returncode}): '
                                'not recording'),
                    Shutdown(reason='no first frame')]
        return [LogInfo(msg=f'first frame arrived; recording to bags/{name} ...'), recorder]

    def after_recording(_event, _context):
        try:
            lines = summarise(bag)
        except RuntimeError as err:
            # Raised inside the launch service, so `ros2 launch` exits non-zero.
            raise RuntimeError(str(err)) from None
        return [LogInfo(msg='\n'.join(lines)), Shutdown(reason='recorded')]

    return [
        LogInfo(msg=(f'record: bags/{name}, {seconds}s. What makes a clip useful: '
                     'docs/info/viewers.md#record')),
        RegisterEventHandler(OnProcessExit(target_action=reset, on_exit=after_reset)),
        RegisterEventHandler(OnProcessExit(target_action=first_frame,
                                           on_exit=after_first_frame)),
        RegisterEventHandler(OnProcessExit(target_action=recorder,
                                           on_exit=after_recording)),
        reset,
        # A backstop, not the mechanism: a camera that never publishes leaves
        # first_frame waiting for ever.
        TimerAction(period=float(seconds + 90),
                    actions=[Shutdown(reason='the recording never finished')]),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('name', description='The directory under bags/ to create.'),
        DeclareLaunchArgument('seconds', default_value='60',
                              description='Seconds of clip, counted from the first frame.'),
        OpaqueFunction(function=_setup),
    ])
