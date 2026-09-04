import unittest

from sensor_msgs.msg import JointState

from xlerobot_task.fetch_deliver_task_node import (
    joints_at_target,
    update_head_stable_since,
)
from xlerobot_task.flow import CAPABILITY_SEQUENCE, FetchDeliverRequest, validate_request


class FlowTest(unittest.TestCase):
    def test_sequence_exposes_task_level_capabilities_only(self):
        self.assertEqual(
            CAPABILITY_SEQUENCE,
            (
                "auto_localize",
                "navigate_to_named_place",
                "detect_object",
                "grasp_object",
                "scan_for_person",
                "approach_target",
                "speak_text",
                "handover_object",
            ),
        )
        self.assertNotIn("dock_to_named_place", CAPABILITY_SEQUENCE)
        self.assertNotIn("pregrasp", CAPABILITY_SEQUENCE)

    def test_required_task_fields(self):
        for field in ("object_id", "source_place", "recipient_id"):
            with self.subTest(field=field):
                values = {
                    "object_id": "yellow_stick",
                    "source_place": "table",
                    "recipient_id": "nearest_person",
                    "grasp_backend": "act",
                    "dry_run": True,
                }
                values[field] = ""
                with self.assertRaises(ValueError):
                    validate_request(FetchDeliverRequest(**values))

    def test_only_runtime_supported_recipient_is_accepted(self):
        with self.assertRaisesRegex(ValueError, "nearest_person"):
            validate_request(FetchDeliverRequest(
                object_id="羽毛球",
                source_place="table",
                recipient_id="person",
                grasp_backend="act",
                dry_run=False,
            ))

    def test_grasp_backend_is_explicit_and_closed_set(self):
        for backend in ("act", "centroid", "gpd"):
            validate_request(FetchDeliverRequest(
                object_id="羽毛球",
                source_place="table",
                recipient_id="nearest_person",
                grasp_backend=backend,
                dry_run=True,
            ))
        for backend in ("", "auto", "traditional"):
            with self.subTest(backend=backend), self.assertRaisesRegex(
                ValueError, "grasp_backend"
            ):
                validate_request(FetchDeliverRequest(
                    object_id="羽毛球",
                    source_place="table",
                    recipient_id="nearest_person",
                    grasp_backend=backend,
                    dry_run=True,
                ))

    def test_head_arrival_uses_measured_joint_positions(self):
        state = JointState(
            name=["head_tilt_joint", "head_pan_joint"],
            position=[0.75, 0.01],
        )
        self.assertTrue(joints_at_target(
            state,
            ["head_pan_joint", "head_tilt_joint"],
            [0.0, 0.8],
            0.06,
        ))
        state.position[0] = 0.70
        self.assertFalse(joints_at_target(
            state,
            ["head_pan_joint", "head_tilt_joint"],
            [0.0, 0.8],
            0.06,
        ))

    def test_head_stability_requires_one_continuous_tolerance_window(self):
        stable_since = update_head_stable_since(None, 1.0, True)
        self.assertEqual(stable_since, 1.0)
        stable_since = update_head_stable_since(stable_since, 1.3, True)
        self.assertEqual(stable_since, 1.0)
        stable_since = update_head_stable_since(stable_since, 1.4, False)
        self.assertIsNone(stable_since)
        stable_since = update_head_stable_since(stable_since, 1.5, True)
        self.assertEqual(stable_since, 1.5)


if __name__ == "__main__":
    unittest.main()
