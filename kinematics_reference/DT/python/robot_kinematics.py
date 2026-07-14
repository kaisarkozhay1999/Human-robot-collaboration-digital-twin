from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Iterable, Sequence


Vector3 = tuple[float, float, float]


@dataclass(frozen=True)
class RobotLinkParameter:
    name: str
    link_length_meters: float
    joint_offset_degrees: float = 0.0


@dataclass(frozen=True)
class RobotKinematicParameters:
    model_name: str
    base_position_meters: Vector3
    horizontal_axis: Vector3
    vertical_axis: Vector3
    links: tuple[RobotLinkParameter, ...]

    @staticmethod
    def from_json(path: str | Path) -> "RobotKinematicParameters":
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        links = tuple(
            RobotLinkParameter(
                name=item.get("name", f"Link{index + 1}"),
                link_length_meters=float(item["linkLengthMeters"]),
                joint_offset_degrees=float(item.get("jointOffsetDegrees", 0.0)),
            )
            for index, item in enumerate(data["links"])
        )

        return RobotKinematicParameters(
            model_name=data.get("modelName", "unnamed_robot_model"),
            base_position_meters=_vector_from_mapping(data.get("basePositionMeters", {})),
            horizontal_axis=_vector_from_mapping(data.get("horizontalAxis", {"x": 1.0, "y": 0.0, "z": 0.0})),
            vertical_axis=_vector_from_mapping(data.get("verticalAxis", {"x": 0.0, "y": 1.0, "z": 0.0})),
            links=links,
        )


@dataclass(frozen=True)
class RobotLinkSegment:
    name: str
    index: int
    start: Vector3
    end: Vector3


@dataclass(frozen=True)
class ForwardKinematicsResult:
    joint_positions: tuple[Vector3, ...]
    link_segments: tuple[RobotLinkSegment, ...]
    end_effector_position: Vector3
    end_effector_angle_degrees: float


@dataclass(frozen=True)
class HumanRobotDistanceResult:
    valid: bool
    distance_meters: float
    human_joint_index: int
    robot_link_index: int
    human_joint_position: Vector3
    closest_point_on_robot_link: Vector3


@dataclass(frozen=True)
class InverseKinematicsResult:
    converged: bool
    joint_angles_degrees: tuple[float, float, float]
    iterations: int
    residual_norm: float


def forward_kinematics(
    parameters: RobotKinematicParameters,
    joint_angles_degrees: Sequence[float],
) -> ForwardKinematicsResult:
    if len(joint_angles_degrees) < len(parameters.links):
        raise ValueError("joint_angles_degrees must contain one angle per configured link")

    horizontal = _normalize(parameters.horizontal_axis)
    vertical = _normalize(parameters.vertical_axis)
    joints: list[Vector3] = [parameters.base_position_meters]
    segments: list[RobotLinkSegment] = []
    cumulative_angle_degrees = 0.0

    for index, link in enumerate(parameters.links):
        cumulative_angle_degrees += joint_angles_degrees[index] + link.joint_offset_degrees
        angle_radians = math.radians(cumulative_angle_degrees)

        # Same serial planar FK as the Mathematica notebook:
        # p_i = p_{i-1} + L_i [cos(sum theta), sin(sum theta)].
        # horizontal_axis and vertical_axis embed the 2D plane in the project frame.
        direction = _add(
            _scale(horizontal, math.cos(angle_radians)),
            _scale(vertical, math.sin(angle_radians)),
        )
        next_joint = _add(joints[-1], _scale(direction, link.link_length_meters))
        joints.append(next_joint)
        segments.append(RobotLinkSegment(link.name or f"Link{index + 1}", index, joints[-2], next_joint))

    return ForwardKinematicsResult(
        joint_positions=tuple(joints),
        link_segments=tuple(segments),
        end_effector_position=joints[-1],
        end_effector_angle_degrees=cumulative_angle_degrees,
    )


def minimum_human_robot_distance(
    human_joints: Iterable[Vector3],
    robot_segments: Sequence[RobotLinkSegment],
) -> HumanRobotDistanceResult:
    best_distance = math.inf
    best_human_index = -1
    best_link_index = -1
    best_human_point = (0.0, 0.0, 0.0)
    best_robot_point = (0.0, 0.0, 0.0)

    for human_index, human_point in enumerate(human_joints):
        if not _is_finite_vector(human_point):
            continue

        for segment in robot_segments:
            distance, closest = point_to_segment_distance(human_point, segment.start, segment.end)
            if distance < best_distance:
                best_distance = distance
                best_human_index = human_index
                best_link_index = segment.index
                best_human_point = human_point
                best_robot_point = closest

    if best_human_index < 0:
        return HumanRobotDistanceResult(False, math.inf, -1, -1, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))

    return HumanRobotDistanceResult(
        True,
        best_distance,
        best_human_index,
        best_link_index,
        best_human_point,
        best_robot_point,
    )


def point_to_segment_distance(point: Vector3, segment_start: Vector3, segment_end: Vector3) -> tuple[float, Vector3]:
    segment = _sub(segment_end, segment_start)
    length_squared = _dot(segment, segment)
    if length_squared <= 1e-12:
        return _distance(point, segment_start), segment_start

    # Project onto the infinite line, then clamp to the physical link segment.
    t = _dot(_sub(point, segment_start), segment) / length_squared
    t = max(0.0, min(1.0, t))
    closest = _add(segment_start, _scale(segment, t))
    return _distance(point, closest), closest


def decide_safety(
    data_is_fresh: bool,
    gesture_stop_requested: bool,
    gesture_go_requested: bool,
    distance_meters: float,
    warning_threshold_meters: float,
    stop_threshold_meters: float,
    release_threshold_meters: float,
    previous_decision: str,
) -> str:
    if not data_is_fresh or gesture_stop_requested or not math.isfinite(distance_meters):
        return "Stop"

    if previous_decision == "Stop" and not gesture_go_requested and distance_meters < release_threshold_meters:
        return "Stop"

    if distance_meters <= stop_threshold_meters:
        return "Stop"

    return "Warning" if distance_meters <= warning_threshold_meters else "Safe"


def solve_planar_ik3(
    parameters: RobotKinematicParameters,
    target_xy_meters: tuple[float, float],
    target_angle_degrees: float,
    initial_guess_degrees: Sequence[float],
    max_iterations: int = 40,
    tolerance: float = 1e-4,
) -> InverseKinematicsResult:
    if len(parameters.links) != 3:
        raise ValueError("Newton-Raphson IK supports exactly the 3-link planar model")

    theta = [float(value) for value in initial_guess_degrees[:3]]
    if len(theta) != 3:
        raise ValueError("initial_guess_degrees must contain exactly three angles")

    residual_norm = math.inf
    for iteration in range(max_iterations):
        fk = forward_kinematics(parameters, theta)
        end_effector_xy = _planar_coordinates(parameters, fk.end_effector_position)
        residual = [
            end_effector_xy[0] - target_xy_meters[0],
            end_effector_xy[1] - target_xy_meters[1],
            _delta_angle_degrees(target_angle_degrees, fk.end_effector_angle_degrees),
        ]
        residual_norm = math.sqrt(sum(value * value for value in residual))
        if residual_norm <= tolerance:
            return InverseKinematicsResult(True, tuple(theta), iteration, residual_norm)

        jacobian = _numerical_jacobian(parameters, theta)
        correction = _solve_3x3(jacobian, [-value for value in residual])
        theta = [theta[index] + correction[index] for index in range(3)]

    return InverseKinematicsResult(False, tuple(theta), max_iterations, residual_norm)


def _numerical_jacobian(parameters: RobotKinematicParameters, theta: Sequence[float]) -> list[list[float]]:
    h = 1e-3
    base = forward_kinematics(parameters, theta)
    base_xy = _planar_coordinates(parameters, base.end_effector_position)
    jacobian = [[0.0, 0.0, 0.0] for _ in range(3)]

    for column in range(3):
        perturbed = list(theta)
        perturbed[column] += h
        fk = forward_kinematics(parameters, perturbed)
        xy = _planar_coordinates(parameters, fk.end_effector_position)
        jacobian[0][column] = (xy[0] - base_xy[0]) / h
        jacobian[1][column] = (xy[1] - base_xy[1]) / h
        jacobian[2][column] = _delta_angle_degrees(base.end_effector_angle_degrees, fk.end_effector_angle_degrees) / h

    return jacobian


def _planar_coordinates(parameters: RobotKinematicParameters, world_point: Vector3) -> tuple[float, float]:
    relative = _sub(world_point, parameters.base_position_meters)
    horizontal = _normalize(parameters.horizontal_axis)
    vertical = _normalize(parameters.vertical_axis)
    return (_dot(relative, horizontal), _dot(relative, vertical))


def _solve_3x3(matrix: Sequence[Sequence[float]], rhs: Sequence[float]) -> list[float]:
    a = [list(row) for row in matrix]
    b = list(rhs)

    for pivot in range(3):
        best_row = max(range(pivot, 3), key=lambda row: abs(a[row][pivot]))
        if abs(a[best_row][pivot]) <= 1e-12:
            raise ValueError("singular IK Jacobian")

        if best_row != pivot:
            a[pivot], a[best_row] = a[best_row], a[pivot]
            b[pivot], b[best_row] = b[best_row], b[pivot]

        for row in range(pivot + 1, 3):
            factor = a[row][pivot] / a[pivot][pivot]
            for column in range(pivot, 3):
                a[row][column] -= factor * a[pivot][column]
            b[row] -= factor * b[pivot]

    x = [0.0, 0.0, 0.0]
    for row in range(2, -1, -1):
        numerator = b[row] - sum(a[row][column] * x[column] for column in range(row + 1, 3))
        x[row] = numerator / a[row][row]

    return x


def _vector_from_mapping(data: dict[str, float]) -> Vector3:
    return (float(data.get("x", 0.0)), float(data.get("y", 0.0)), float(data.get("z", 0.0)))


def _normalize(vector: Vector3) -> Vector3:
    magnitude = math.sqrt(_dot(vector, vector))
    if magnitude <= 1e-12:
        raise ValueError("axis vectors must be non-zero")
    return (vector[0] / magnitude, vector[1] / magnitude, vector[2] / magnitude)


def _is_finite_vector(vector: Vector3) -> bool:
    return all(math.isfinite(value) for value in vector)


def _delta_angle_degrees(current: float, target: float) -> float:
    return (target - current + 180.0) % 360.0 - 180.0


def _add(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _sub(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _scale(a: Vector3, scalar: float) -> Vector3:
    return (a[0] * scalar, a[1] * scalar, a[2] * scalar)


def _dot(a: Vector3, b: Vector3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _distance(a: Vector3, b: Vector3) -> float:
    return math.sqrt(sum((a[index] - b[index]) ** 2 for index in range(3)))
