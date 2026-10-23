"""Both launch entrypoints retain run.py as sole run/process owner."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration


def description(root, reuse):
    def start(context):
        value = lambda key: LaunchConfiguration(key).perform(context)
        command = [str(root / 'tools/run.sh'), '--config', str(root / 'packages/liuo_tunnel/config' / ('phase3_reuse_map.json' if reuse else 'phase3_self_calibrate.json')),
                   '--duration', value('duration'), '--seed', value('seed'), '--wall-timeout', value('wall_timeout'), '--visualize']
        if reuse:
            path = Path(value('map')).expanduser().resolve()
            if not path.is_file():
                raise RuntimeError('map:= must point to a first-run export/map/calibrated_map.json')
            command += ['--map', str(path)]
        else:
            command += ['--installation', str(root / 'packages/liuo_tunnel/config/installation/straight_simulated.json')]
        if value('replay'):
            command += ['--replay', str(Path(value('replay')).expanduser().resolve())]
        else:
            command += ['--calibration-route', '--sim-start', *value('sim_start').split()]
        for key in ('rviz', 'capture', 'formal'):
            if value(key).lower() == 'true':
                command.append('--' + key)
        process = ExecuteProcess(cmd=command, cwd=str(root), output='screen', sigterm_timeout='15', sigkill_timeout='25')

        def exited(event, context):
            if event.returncode != 0:
                raise RuntimeError(f'LIUO run failed ({event.returncode}); inspect the printed run directory')
            return []

        return [RegisterEventHandler(OnProcessExit(target_action=process, on_exit=exited)), process]

    defaults = dict(duration='360', seed='1' if reuse else '0', wall_timeout='600', rviz='true', capture='false', formal='false', replay='',
                    sim_start='220 0.2 1.6 1.5707963267948966' if reuse else '200 0 1.2 0')
    if reuse:
        defaults['map'] = ''
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()] + [OpaqueFunction(function=start)])
