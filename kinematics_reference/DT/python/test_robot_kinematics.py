import math
import unittest

try:
    from robot_kinematics import (
        RobotKinematicParameters,
        RobotLinkParameter,
        decide_safety,
        forward_kinematics,
        minimum_human_robot_distance,
        point_to_segment_distance,
    )
except ModuleNotFoundError:
    from .robot_kinematics import (
        RobotKinematicParameters,
        RobotLinkParameter,
        decide_safety,
        forward_kinematics,
        minimum_human_robot_distance,
        point_to_segment_distance,
    )


def parameters() -> RobotKinematicParameters:
    return RobotKinematicParameters(
        model_name="test",
        base_position_meters=(0.0, 0.0, 0.0),
        horizontal_axis=(1.0, 0.0, 0.0),
        vertical_axis=(0.0, 1.0, 0.0),
        links=(
            RobotLinkParameter("L1", 0.25),
            RobotLinkParameter("L2", 0.20),
            RobotLinkParameter("L3", 0.15),
        ),
    )


class RobotKinematicsTests(unittest.TestCase):
    def test_forward_kinematics_straight_arm(self) -> None:
        fk = forward_kinematics(parameters(), (0.0, 0.0, 0.0))
        self.assertAlmostEqual(fk.end_effector_position[0], 0.60, places=6)
        self.assertAlmostEqual(fk.end_effector_position[1], 0.0, places=6)
        self.assertEqual(len(fk.link_segments), 3)

    def test_point_to_segment_distance_clamps_to_segment(self) -> None:
        distance, closest = point_to_segment_distance((0.5, 0.3, 0.0), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
        self.assertAlmostEqual(distance, 0.3, places=6)
        self.assertAlmostEqual(closest[0], 0.5, places=6)

    def test_minimum_human_robot_distance_ignores_invalid_points(self) -> None:
        fk = forward_kinematics(parameters(), (0.0, 0.0, 0.0))
        result = minimum_human_robot_distance([(math.nan, 0.0, 0.0), (0.3, 0.1, 0.0)], fk.link_segments)
        self.assertTrue(result.valid)
        self.assertAlmostEqual(result.distance_meters, 0.1, places=6)
        self.assertEqual(result.human_joint_index, 1)

    def test_safety_hysteresis_and_stale_data(self) -> None:
        self.assertEqual(
            decide_safety(True, False, False, 0.30, 0.45, 0.25, 0.35, "Stop"),
            "Stop",
        )
        self.assertEqual(
            decide_safety(True, False, False, 0.40, 0.45, 0.25, 0.35, "Stop"),
            "Warning",
        )
        self.assertEqual(
            decide_safety(False, False, False, 10.0, 0.45, 0.25, 0.35, "Safe"),
            "Stop",
        )


if __name__ == "__main__":
    unittest.main()
