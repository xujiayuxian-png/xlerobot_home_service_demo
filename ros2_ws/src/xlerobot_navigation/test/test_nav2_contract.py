import importlib.util
from pathlib import Path

import yaml


def _load_named_launch():
    path = Path(__file__).resolve().parents[1] / 'launch' / 'named_navigation.launch.py'
    spec = importlib.util.spec_from_file_location('named_navigation_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_named_places_file_is_translated_to_server_parameters(tmp_path):
    places = tmp_path / 'places.yaml'
    places.write_text(
        'named_places:\n'
        '  frame_id: map\n'
        '  places:\n'
        '    table:\n'
        '      x: 1.0\n'
        '      y: 2.0\n'
        '      yaw: -0.5\n'
        '      nav_offset_m: 0.4\n',
        encoding='utf-8',
    )
    parameters = _load_named_launch()._place_parameters(str(places))

    assert parameters['place_ids'] == ['table']
    assert parameters['places.table.x'] == 1.0
    assert parameters['places.table.nav_offset_m'] == 0.4
    assert parameters['places.table.dock'] is True


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_nav2_owns_position_but_not_final_yaw():
    config = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'nav2_two_wheel_reference.yaml').read_text()
    )
    controller = config['controller_server']['ros__parameters']
    goal = controller['general_goal_checker']
    follow = controller['FollowPath']
    assert goal['xy_goal_tolerance'] == 0.04
    assert goal['yaw_goal_tolerance'] == 3.15
    assert follow['rotate_to_goal_heading'] is False


def test_reference_navigation_accepts_narrow_passages_with_bounded_recovery():
    config = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'nav2_two_wheel_reference.yaml').read_text()
    )
    controller = config['controller_server']['ros__parameters']
    local = config['local_costmap']['local_costmap']['ros__parameters']
    global_costmap = config['global_costmap']['global_costmap']['ros__parameters']
    planner = config['planner_server']['ros__parameters']
    assert controller['progress_checker']['movement_time_allowance'] == 10.0
    assert global_costmap['update_frequency'] == 1.0
    navigator = config['bt_navigator']['ros__parameters']
    assert planner['expected_planner_frequency'] == 1.0
    assert navigator['default_server_timeout'] == 500
    assert local['footprint_padding'] == 0.005
    assert local['inflation_layer']['inflation_radius'] == 0.22
    assert global_costmap['inflation_layer']['inflation_radius'] == 0.30
    assert global_costmap['inflation_layer']['cost_scaling_factor'] == 5.0

    tree = (PACKAGE_ROOT / 'behavior_trees' / 'navigate_to_pose_position_only.xml').read_text()
    assert '<RateController hz="1.0">' in tree
    recovery = tree.split('<RoundRobin name="RecoveryActions">', 1)[1]
    assert recovery.index('<BackUp ') < recovery.index('<Sequence name="ClearingActions">')


def test_table_dock_accepts_eight_centimeters_lateral_error():
    config = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'named_navigation.yaml').read_text()
    )
    parameters = config['named_navigation_server']['ros__parameters']
    assert parameters['dock.lateral_tolerance_m'] == 0.08
    assert 'dock.max_abs_lateral_error_m' not in parameters


def test_only_collision_monitor_publishes_final_nav_topic():
    config = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'nav2_two_wheel_reference.yaml').read_text()
    )
    monitor = config['collision_monitor']['ros__parameters']
    assert monitor['cmd_vel_in_topic'] == 'cmd_vel_smoothed'
    assert monitor['cmd_vel_out_topic'] == 'cmd_vel_nav'

    launch_source = (
        PACKAGE_ROOT / 'launch' / 'nav2_navigation.launch.py'
    ).read_text()
    raw_remap = """remappings=[('cmd_vel', 'cmd_vel_nav_raw')]"""
    assert launch_source.count(raw_remap) == 3
    assert 'cmd_vel_nav")' not in launch_source


def test_auto_localizer_delegates_motion_to_nav2_spin():
    source = (PACKAGE_ROOT / 'src' / 'auto_localizer_server.cpp').read_text()
    assert 'rclcpp_action::create_client<Spin>' in source
    assert 'create_client<ManageLifecycleNodes>' in source
    assert 'ensure_navigation_active' in source
    assert 'create_publisher' not in source
    assert 'geometry_msgs/msg/twist' not in source


def test_spin_motion_is_active_before_map_dependent_navigation():
    launch_source = (
        PACKAGE_ROOT / 'launch' / 'nav2_navigation.launch.py'
    ).read_text()
    motion_start = launch_source.index('localization_motion_nodes = [')
    motion_end = launch_source.index(']', motion_start)
    motion_group = launch_source[motion_start:motion_end]
    navigation_start = launch_source.index('navigation_nodes = [')
    navigation_end = launch_source.index(']', navigation_start)
    navigation_group = launch_source[navigation_start:navigation_end]
    for node in ('behavior_server', 'velocity_smoother', 'collision_monitor'):
        assert f"'{node}'" in motion_group
        assert f"'{node}'" not in navigation_group
    for node in ('planner_server', 'bt_navigator'):
        assert f"'{node}'" not in motion_group
        assert f"'{node}'" in navigation_group
    assert "{'autostart': True, 'node_names': localization_motion_nodes}" in launch_source
    assert "{'autostart': False, 'node_names': navigation_nodes}" in launch_source


def test_amcl_launch_requires_an_explicit_map():
    launch_source = (
        PACKAGE_ROOT / 'launch' / 'amcl_localization.launch.py'
    ).read_text()
    assert "DeclareLaunchArgument(\n                'map', description=" in launch_source
    assert "'map', default_value=" not in launch_source


def test_mapping_uses_reference_frames_and_scan_topic():
    config = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'slam_toolbox_mapping.yaml').read_text()
    )['slam_toolbox']['ros__parameters']
    assert config['mode'] == 'mapping'
    assert (config['map_frame'], config['odom_frame'], config['base_frame']) == (
        'map',
        'odom',
        'base_link',
    )
    assert config['scan_topic'] == '/scan'
    assert config['resolution'] == 0.03
