"""Opt-in X1 launch ordering. Probes observe readiness; they never send goals."""

import json

from launch.actions import LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# Keep this list explicit: a new demo component must choose a startup phase.
PHASE_KEYS = (
    ('platform', ('platform_runtime.launch.py', 'robot_state_gateway')),
    ('sensors', ('sensors.launch.py', 'wrist_camera_native', 'camera_health')),
    ('navigation_moveit', (
        'amcl_localization.launch.py', 'nav2_navigation.launch.py',
        'scan_map_consistency.launch.py', 'auto_localizer.launch.py',
        'named_navigation.launch.py', 'approach_target.launch.py', 'move_group.launch.py')),
    ('perception', ('detect_object.launch.py', 'verify_grasp.launch.py',
                    'scan_for_person.launch.py', 'act_policy_adapter')),
    ('application', ('streaming_executor.launch.py', 'person_search_node',
                     'grasp_object.launch.py', 'handover_object.launch.py',
                     'speak_text.launch.py', 'fetch_deliver_task.launch.py',
                     'voice_assistant', 'operator_console', 'operator_exit_guard')),
)

REQUIREMENTS = {
    'platform': {'diagnostics': ['xlerobot/startup_ready', 'xlerobot/drive_safety'],
                 'services': ['/x1/lookup_transform']},
    'sensors': {'diagnostics': ['xlerobot/camera/' + name for name in
                              ('head_color', 'head_depth', 'head_camera_info', 'wrist')]},
    # Planner activation and localization belong to a task, not to startup.
    'navigation_moveit': {
        'diagnostics': ['lifecycle_manager_localization: Nav2 Health',
                        'lifecycle_manager_localization_motion: Nav2 Health'],
        'services': ['/compute_ik', '/auto_localize/_action/send_goal',
                     '/navigate_to_named_place/_action/send_goal',
                     '/approach_target/_action/send_goal']},
    'perception': {'model_service': '/scan_for_person/model_ready',
                   'services': ['/detect_object/_action/send_goal',
                                '/verify_grasp/_action/send_goal',
                                '/execute_learned_policy/_action/send_goal',
                                '/scan_for_person_view/_action/send_goal']},
}


def advance_after_probe(event, context, next_actions, phase):
    if context.is_shutdown:
        return None
    if event.returncode != 0:
        raise RuntimeError(f'X1 startup phase {phase} failed (exit {event.returncode}); '
                           'stopping demo; explicit restart required')
    return next_actions


def chain_phases(phases, probe_factory):
    """Build backwards so handlers exist before their probe can exit."""
    following = []
    for name, actions in reversed(phases):
        started = [LogInfo(msg=f'X1 startup phase: {name}'), *actions]
        if name == 'application':
            following = [*started, LogInfo(msg='X1 application launched; '
                                          'task and voice readiness remain independently checked')]
            continue
        probe = probe_factory(name, REQUIREMENTS[name])
        next_actions = following
        handler = RegisterEventHandler(OnProcessExit(
            target_action=probe,
            on_exit=lambda event, context, next_actions=next_actions, phase=name:
                advance_after_probe(event, context, next_actions, phase)))
        following = [handler, *started, probe]
    return following


def staged_demo_actions(actions, context):
    groups = {name: [] for name, _ in PHASE_KEYS}
    owners = {key: name for name, keys in PHASE_KEYS for key in keys}
    for action in actions:
        key = getattr(action, 'x1_startup_key', None)
        if key is None and isinstance(action, Node):
            key = perform_substitutions(context, normalize_to_list_of_substitutions(action.node_executable))
        if key not in owners:
            raise ValueError(f'X1 startup phase missing for {key or type(action).__name__}')
        groups[owners[key]].append(action)

    def probe_factory(name, requirements):
        return Node(package='xlerobot_bringup', executable='startup_probe',
                    name=f'x1_startup_probe_{name}', output='screen',
                    parameters=[{'stage_spec': ParameterValue(
                        json.dumps({'name': name, **requirements}), value_type=str)}])

    return chain_phases(list(groups.items()), probe_factory)
