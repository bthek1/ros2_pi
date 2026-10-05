"""A viewer: the pipeline, a source of frames, and RViz — one file for every view.

`just view-mesh` used to be a 135-line shell script, and so did seven siblings
that differed from it in an RViz config name, a sleep and some checklist text.
What a view *is* now lives in `config/views.yaml`, one entry per view, and this
file is the one place that turns an entry into processes. #15's P0.

**What `launch` does here that the scripts did by hand**:

- *Which config, which source* are launch arguments — `view:=`, `bag:=` — instead
  of positional `$1`/`$2` parsed again in every script.
- *`sleep 8` before RViz* is a `TimerAction`, with the delay read from the view.
- *The window was closed* is `OnProcessExit(rviz) -> Shutdown()`. That ending is
  the one `gates/teardown.sh` calls CLOSE, and it is the one that leaked on
  2026-09-14: in bash it was a `wait` returning and an EXIT trap running from an
  ordinary end of script, a different code path from every signal. Here it is an
  event handler, and `close_ends_session:=false` removes it so the gate can watch
  CLOSE fail without it.
- *`seconds`* is a `TimerAction` that shuts the whole launch down, rather than a
  `timeout` wrapped around each process separately.
- *Shutdown* sends SIGINT to every child and escalates to SIGTERM and SIGKILL by
  itself, which is the part of `kill_local` that existed because bash owned the
  process tree.

**What it does not do, deliberately: tear down the Pi.** `launch` has no notion
of a second machine. The camera is started over ssh, and killing the local ssh
client does **not** reach the far end — measured 2026-09-14, the remote
`timeout`/`ros2 run`/`camera_node` chain carried on after its client died. So a
session is always `tools/session.sh <label> -- ros2 launch ...`, and that
wrapper's `kill_pi` is the teardown of the far end, verified, exactly as before.
Launching this file bare works and is fine on a bag; on the camera it leaves the
Pi to its own `timeout`.

**`pimesh.launch.py` is included, never redeclared.** Every argument it declares
— `odom_regime`, `local_ba`, `loop_closure`, `dashboard_port`, thirty of them —
is forwarded from this file's command line untouched, because an include sees its
parent's launch configurations. A view that needs one set (the replay view wants
`pipeline:=false`) says so in `views.yaml`, and this file sets it before the
include runs. Declaring any of them again here would be two defaults for one
value, which is the drift `test_no_launch_override_quietly_replaces_what_the_yaml_says`
exists for one layer down.

**The RViz process is `/opt/ros/<distro>/lib/rviz2/rviz2`, not `rviz2`.**
`launch_ros`'s `Node` resolves the executable to its installed path before exec,
so argv[0] carries the directory. `PIMESH_VIEWER_PAT` in `tools/lib/just-lib.sh`
was anchored on a bare `rviz2` at the start of the command line, and would have
matched no RViz this file starts — the teardown sweep would have reported a
leaked window as clean, and `gates/teardown.sh` could not have closed one.
Measured 2026-10-05 before the first line of this file was written.
"""

import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler, SetLaunchConfiguration,
                            Shutdown, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SHARE = get_package_share_directory('pimesh_bringup')
VIEWS_PATH = os.path.join(SHARE, 'config', 'views.yaml')

MODEL_RELPATH = 'models/depth_anything_v2_small.onnx'


def load_views(path: str = VIEWS_PATH) -> dict:
    """The views table. A function so test_views can read the same file the same way."""
    with open(path) as fh:
        return yaml.safe_load(fh)


def workspace() -> Path:
    """The workspace root: where bags/, models/ and tools/ are.

    `tools/session.sh` exports PIMESH_WS, and that wins. Without it the root is
    derived from this package's share directory, which colcon puts at
    `<ws>/install/pimesh_bringup/share/pimesh_bringup` — true of this workspace's
    isolated install and stated as an assumption rather than hidden in a path.
    """
    env = os.environ.get('PIMESH_WS')
    if env:
        return Path(env)
    return Path(SHARE).resolve().parents[3]


def resolve_bag(arg: str, ws: Path) -> Path:
    """A bag name under bags/ or a path to one, judged by its metadata.yaml.

    The same rule every viewer script had, and the same refusal: a directory
    without metadata.yaml is a recording that was not finalised, and `ros2 bag
    play` on it fails in a way that reads as the pipeline's fault.
    """
    for cand in (Path(arg), ws / 'bags' / arg):
        if (cand / 'metadata.yaml').is_file():
            return cand.resolve()
    raise RuntimeError(
        f"no bag at '{arg}' (looked for metadata.yaml there and under {ws / 'bags'})")


def _setup(context):
    views = load_views()
    name = LaunchConfiguration('view').perform(context)
    if name not in views:
        # An unknown view must not open an empty RViz: that looks like a pipeline
        # with nothing in it, which is a different and much more alarming fault.
        raise RuntimeError(
            f"no view '{name}'. Valid views: {', '.join(sorted(views))} "
            f'(config/views.yaml)')
    view = views[name]
    ws = workspace()
    seconds = float(LaunchConfiguration('seconds').perform(context))
    bag_arg = LaunchConfiguration('bag').perform(context)

    if view.get('needs_model') and not (ws / MODEL_RELPATH).is_file():
        raise RuntimeError(
            f'no model at {ws / MODEL_RELPATH}. models/ is git-ignored; '
            'fetch it with: bash tools/fetch-model.sh')
    if view.get('needs_bag') and not bag_arg:
        raise RuntimeError(f"view '{name}' plays a bag: give it bag:=<name or path>")
    if view.get('loop') and view.get('launch', {}).get('pipeline') != 'false':
        # test_views asserts this too. It is checked here as well because a
        # looping bag under a live pipeline does not fail: it freezes the pose and
        # floods every TF listener, and looks like RViz being slow.
        raise RuntimeError(f"view '{name}' loops its bag and so must set pipeline: 'false'")

    actions = []

    # The view's own pimesh.launch.py arguments, set before the include below is
    # visited — launch visits actions depth-first in order, so these land first.
    for key, value in view.get('launch', {}).items():
        actions.append(SetLaunchConfiguration(key, str(value)))

    if bag_arg:
        bag = resolve_bag(bag_arg, ws)
        source = f'{bag}' + (' — looping' if view.get('loop') else ' — one pass, not looped')
        cmd = ['ros2', 'bag', 'play', str(bag), '--disable-keyboard-controls']
        if view.get('loop'):
            cmd.append('--loop')
        actions.append(ExecuteProcess(cmd=cmd, name='bag_play', output='log'))
    else:
        source = 'the live camera on the Pi'
        # pi_run_for, from the one file that spells the Pi's ssh: `timeout` goes
        # inside the remote login shell so the limit reaches the node, not the
        # shell. The local half of this is an ssh client that launch can kill and
        # the far end will not notice — session.sh's kill_pi is what ends it.
        actions.append(ExecuteProcess(
            cmd=['bash', '-c',
                 # The explicit "" matters: a sourced file sees the caller's
                 # positional parameters, and just-lib.sh would read $1 — the
                 # workspace path — as an option and exit (found by #15's P1).
                 'source "$1/tools/lib/just-lib.sh" "" && '
                 # `& wait`, not a foreground child: launch signals this bash
                 # alone, and bash defers SIGINT until a foreground child exits —
                 # so every camera session sat out launch's 5 s before SIGTERM.
                 # The orphaned ssh is kill_local's; the far end is kill_pi's.
                 'pi_run_for "$2" "ros2 run pimesh_camera camera_node" & wait $!',
                 'pi_camera', str(ws), str(int(seconds))],
            name='pi_camera', output='log'))

    rviz_cfg = view.get('rviz')
    if rviz_cfg:
        rviz = Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', os.path.join(SHARE, 'rviz', rviz_cfg)],
            # Wayland session; rviz2 renders through GLX and needs xcb.
            additional_env={'QT_QPA_PLATFORM': 'xcb'},
            output='log')
        actions.append(TimerAction(period=float(view.get('rviz_delay_s', 0.0)),
                                   actions=[rviz]))
        actions.append(RegisterEventHandler(
            OnProcessExit(target_action=rviz,
                          on_exit=[Shutdown(reason='the RViz window was closed')]),
            condition=IfCondition(LaunchConfiguration('close_ends_session'))))

    actions.append(TimerAction(period=seconds,
                               actions=[Shutdown(reason=f'{seconds:g} s elapsed')]))
    context.launch_configurations['_view_source'] = source
    return actions


def _announce(context):
    """The one line a session prints about itself, after the include has run.

    After, because a view without a window has to say where to look instead, and
    the port it would name is `pimesh.launch.py`'s argument: declared — and so
    defaulted — only once the include has been visited.
    """
    name = LaunchConfiguration('view').perform(context)
    cfg = context.launch_configurations
    where = ''
    if cfg.get('dashboard', 'false').lower() == 'true':
        where = f' Open http://localhost:{cfg["dashboard_port"]}.'
    return [LogInfo(msg=(f'view-{name}: source {cfg["_view_source"]}.{where} What to look '
                         f'at: docs/info/viewers.md#view-{name}'))]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'view',
            description='Which entry of config/views.yaml to bring up.'),
        DeclareLaunchArgument(
            'bag', default_value='',
            description='A bag under bags/, or a path to one. Empty uses the Pi\'s '
                        'live camera. Run through tools/session.sh, or the camera '
                        'outlives this launch on the Pi until its own timeout.'),
        DeclareLaunchArgument(
            'seconds', default_value='600',
            description='Shut the whole session down after this long.'),
        DeclareLaunchArgument(
            'close_ends_session', default_value='true',
            description='Closing the RViz window ends the session. false is '
                        "tools/gates/teardown.sh's control: without the handler, "
                        'CLOSE must fail.'),
        OpaqueFunction(function=_setup),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(SHARE, 'launch', 'pimesh.launch.py'))),
        OpaqueFunction(function=_announce),
    ])
