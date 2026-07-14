from __future__ import annotations

from pathlib import Path

from robot_kinematics import RobotKinematicParameters, forward_kinematics, minimum_human_robot_distance


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    parameters = RobotKinematicParameters.from_json(root / "Config" / "robot_kinematics.placeholder.json")
    sample_human_point = (0.30, 0.10, 0.0)
    test_angles = [
        (0.0, 0.0, 0.0),
        (45.0, -20.0, 15.0),
        (100.0, 20.0, 30.0),
    ]

    print(f"Kinematic validation for {parameters.model_name}")
    for angles in test_angles:
        fk = forward_kinematics(parameters, angles)
        distance = minimum_human_robot_distance([sample_human_point], fk.link_segments)
        print(f"angles_deg={angles}")
        for index, position in enumerate(fk.joint_positions):
            print(f"  joint[{index}]={_format_vector(position)} m")
        print(f"  end_effector={_format_vector(fk.end_effector_position)} m phi={fk.end_effector_angle_degrees:.2f} deg")
        print(
            "  min_distance_to_sample="
            f"{distance.distance_meters:.4f} m human_joint={distance.human_joint_index} robot_link={distance.robot_link_index}"
        )


def _format_vector(vector: tuple[float, float, float]) -> str:
    return f"({vector[0]:.4f}, {vector[1]:.4f}, {vector[2]:.4f})"


if __name__ == "__main__":
    main()
