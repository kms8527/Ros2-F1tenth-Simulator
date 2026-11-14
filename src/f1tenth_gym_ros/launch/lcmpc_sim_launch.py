"""F1TENTH simulator whose plant is the MDN-LCMPC prediction model.

    ros2 launch f1tenth_gym_ros lcmpc_sim_launch.py

The ego/opponent plant is ``lcmpc_st``: the ORIGINALMODEL single-track ODE of
MDN_SLCMPC_module/control/MPC/src/dynamics.cpp, driven directly by the MPC's
``[steering_angle, acceleration]`` command. Vehicle constants are read from the
MPC params file so simulator and controller can never drift apart.

Environment overrides:
    LCMPC_PARAMS_FILE      MPC params.yaml          (default: repo params.yaml)
    LCMPC_PATH_FILE        MPC waypoint csv used for spawning (default map1_path.csv)
    LCMPC_SIM_MAP          map basename without .yaml (default maps/map1/map1)
    LCMPC_SIM_NUM_AGENTS   1 or 2                   (default 2)
    LCMPC_OPP_SPAWN_DISTANCE_M   opponent arc-length lead (default 2.5)
    LCMPC_EGO_SPAWN_INDEX  waypoint index of the ego spawn (default 0)
    F1TENTH_SIM_ENABLE_RVIZ  true/false             (default true)
"""

import csv
import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, EnvironmentVariable
from launch_ros.actions import Node

MDN_ROOT = os.environ.get('MDN_SLCMPC_ROOT', '/home/a/MDN_SLCMPC_module')
DEFAULT_PARAMS = os.path.join(MDN_ROOT, 'control', 'MPC', 'params', 'params.yaml')
DEFAULT_PATH = os.path.join(MDN_ROOT, 'control', 'MPC', 'data', 'path', 'map1_path.csv')


def _workspace(package_share):
    return os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(package_share))))


def _lcmpc_plant_params(params_file):
    with open(params_file, 'r') as stream:
        p = yaml.safe_load(stream)['/**']['ros__parameters']
    return {
        'lcmpc_mass': float(p['mass']),
        'lcmpc_l_f': float(p['l_cg2front']),
        'lcmpc_l_r': float(p['l_cg2rear']),
        'lcmpc_h_cg': float(p['height_cg']),
        'lcmpc_I_z': float(p['moment_inertia']),
        'lcmpc_mu': float(p['friction_coeff']),
        'lcmpc_C_Sf': float(p['C_S_front']),
        'lcmpc_C_Sr': float(p['C_S_rear']),
        'lcmpc_Cm0': float(p['Cm0']),
        'lcmpc_steer_max': float(p['STEER_MAX']),
        'lcmpc_accel_max': float(p['ACCELERATION_MAX']),
        'lcmpc_decel_max': float(p['DECELERATION_MAX']),
        'lcmpc_vel_threshold': float(p['VEL_THRESHOLD']),
        'lcmpc_width': float(p['width']),
        'lcmpc_length': float(p['length']),
        'max_steer': float(p['STEER_MAX']),
    }


def _spawn_poses(path_file, ego_index, separation):
    with open(path_file, newline='') as stream:
        rows = [r for r in csv.reader(stream) if r]
    pts = []
    for r in rows:
        try:
            pts.append((float(r[0]), float(r[1])))
        except ValueError:
            continue
    n = len(pts)

    def pose(i):
        x, y = pts[i % n]
        nx, ny = pts[(i + 1) % n]
        return x, y, math.atan2(ny - y, nx - x)

    i, arc = ego_index, 0.0
    while arc < separation and i < ego_index + n // 2:
        arc += math.dist(pts[i % n], pts[(i + 1) % n])
        i += 1
    return pose(ego_index), pose(i)


def generate_launch_description():
    share = get_package_share_directory('f1tenth_gym_ros')
    config = os.path.join(share, 'config', 'lcmpc_sim.yaml')
    cfg = yaml.safe_load(open(config, 'r'))['bridge']['ros__parameters']
    workspace = _workspace(share)

    params_file = os.environ.get('LCMPC_PARAMS_FILE', DEFAULT_PARAMS)
    plant = _lcmpc_plant_params(params_file)

    map_path = os.environ.get('LCMPC_SIM_MAP', cfg['map_path'])
    if not os.path.isabs(map_path):
        map_path = os.path.join(workspace, map_path)
    with open(map_path + '.yaml', 'r') as stream:
        map_yaml = yaml.safe_load(stream)
    map_ext = os.path.splitext(str(map_yaml['image']))[1]

    num_agents = int(os.environ.get('LCMPC_SIM_NUM_AGENTS', cfg['num_agent']))
    ego, opp = _spawn_poses(
        os.environ.get('LCMPC_PATH_FILE', DEFAULT_PATH),
        int(os.environ.get('LCMPC_EGO_SPAWN_INDEX', '0')),
        float(os.environ.get('LCMPC_OPP_SPAWN_DISTANCE_M', '2.5')))
    spawn = {'sx': ego[0], 'sy': ego[1], 'stheta': ego[2],
             'sx1': opp[0], 'sy1': opp[1], 'stheta1': opp[2],
             'initial_speed': 0.0}

    print('[lcmpc_sim] plant params from ' + params_file)
    print('[lcmpc_sim] ' + ', '.join('%s=%g' % kv for kv in plant.items()))
    print('[lcmpc_sim] map=%s ego=(%.3f, %.3f, %.3f) opp=(%.3f, %.3f, %.3f) agents=%d' % (
        map_path, *ego, *opp, num_agents))

    gym_source = os.path.join(workspace, 'src', 'f1tenth_gym')
    bridge = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='bridge',
        parameters=[config, plant, spawn, {
            'map_path': map_path, 'map_img_ext': map_ext,
            'num_agent': num_agents, 'dynamics_model': 'lcmpc_st',
            'use_sim_time': False, 'use_sim_time_bridge': cfg['use_sim_time'],
        }],
        additional_env={
            'PYTHONPATH': gym_source + os.pathsep + os.environ.get('PYTHONPATH', ''),
            'NUMBA_CACHE_DIR': os.environ.get('NUMBA_CACHE_DIR', '/tmp/f1tenth_numba_cache'),
        },
        output='screen',
    )

    # Map server (copy of the image next to a generated yaml, as in gym_bridge_launch).
    temp_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'temp_lcmpc')
    os.makedirs(temp_dir, exist_ok=True)
    scaled = dict(map_yaml)
    scaled['image'] = 'scaled_map' + map_ext
    temp_yaml = os.path.join(temp_dir, 'scaled_map.yaml')
    with open(temp_yaml, 'w') as f:
        yaml.dump(scaled, f)
    with open(os.path.join(temp_dir, 'scaled_map' + map_ext), 'wb') as f, \
            open(map_path + map_ext, 'rb') as g:
        f.write(g.read())

    map_server = Node(
        package='nav2_map_server', executable='map_server',
        parameters=[{'yaml_filename': temp_yaml}, {'topic': 'map'},
                    {'frame_id': 'map'}, {'use_sim_time': False}])
    lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        parameters=[{'use_sim_time': False}, {'autostart': True},
                    {'node_names': ['map_server']}])

    def rsp(name, xacro):
        return Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name=name + '_robot_state_publisher',
            parameters=[{'robot_description': Command(
                ['xacro ', os.path.join(share, 'launch', xacro)])}],
            remappings=[('/robot_description', name + '_robot_description')])

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz',
        arguments=['-d', os.path.join(share, 'launch', 'gym_bridge.rviz')],
        condition=IfCondition(EnvironmentVariable(
            'F1TENTH_SIM_ENABLE_RVIZ', default_value='true')))

    ld = LaunchDescription([rviz, bridge, map_server,
                            TimerAction(period=1.0, actions=[lifecycle]),
                            rsp('ego', 'ego_racecar.xacro')])
    if num_agents > 1:
        ld.add_action(rsp('opp', 'opp_racecar.xacro'))
    return ld
