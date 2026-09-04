from xlerobot_hmi.manual_control import ManualControlCoordinator


def test_action_and_teleop_are_mutually_exclusive():
    coordinator = ManualControlCoordinator()
    action_owner = coordinator.begin_action()
    assert action_owner is not None
    assert coordinator.claim_teleop(object(), task_active=False) is False
    assert coordinator.finish_action(action_owner)
    teleop_owner = object()
    assert coordinator.claim_teleop(teleop_owner, task_active=False)
    assert coordinator.begin_action() is None
    assert coordinator.release_teleop(teleop_owner)


def test_only_the_owner_can_change_action_or_teleop_state():
    coordinator = ManualControlCoordinator()
    action_owner = coordinator.begin_action()
    assert action_owner is not None
    assert coordinator.set_goal_future(object(), object()) is False
    assert coordinator.finish_action(object()) is False
    assert coordinator.action_active()
    assert coordinator.finish_action(action_owner)
    teleop_owner = object()
    assert coordinator.claim_teleop(teleop_owner, task_active=False)
    assert coordinator.release_teleop(object()) is False
    assert coordinator.teleop_active()
    assert coordinator.release_teleop(teleop_owner)


def test_reservation_keeps_manual_control_fail_closed():
    coordinator = ManualControlCoordinator()
    coordinator.set_reserved(True)
    assert coordinator.reserved()
    assert not coordinator.idle(task_active=False)
    assert not coordinator.claim_teleop(object(), task_active=False)
    coordinator.set_reserved(False)
    assert coordinator.idle(task_active=False)


def test_cancel_is_latched_until_owned_action_finishes():
    coordinator = ManualControlCoordinator()
    owner = coordinator.begin_action()
    assert owner is not None
    coordinator.request_cancel()
    assert coordinator.cancel_requested()
    assert coordinator.finish_action(owner)
    assert not coordinator.cancel_requested()
    assert coordinator.idle(task_active=False)


def test_drive_stop_state_is_explicitly_unknown_until_observed():
    coordinator = ManualControlCoordinator()
    assert coordinator.drive_stop_latched is None
    coordinator.set_drive_stop_latched(True)
    assert coordinator.drive_stop_latched is True
    coordinator.set_drive_stop_latched(False)
    assert coordinator.drive_stop_latched is False
