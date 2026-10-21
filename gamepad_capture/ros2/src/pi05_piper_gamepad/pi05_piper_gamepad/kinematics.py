"""URDF-driven FK and bounded numerical IK for the original six-axis PiPER.

This module deliberately reads the checked-in ``piper`` URDF at runtime. It
does not contain a second, hand-written set of robot link dimensions.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np


JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))
DEFAULT_TOOL_TRANSLATION_M = (0.0, 0.0, 0.0)
ORIENTATION_METERS_PER_RADIAN = 0.1
MAX_JACOBIAN_CONDITION = 1e4

Vector3 = tuple[float, float, float]
Matrix3 = tuple[tuple[float, float, float], ...]


@dataclass(frozen=True)
class Pose:
    position_m: Vector3
    rotation: Matrix3


@dataclass(frozen=True)
class _Joint:
    name: str
    xyz_m: Vector3
    rpy_rad: Vector3
    axis: Vector3
    lower_rad: float
    upper_rad: float


def _vec3(text: str | None, default: Vector3) -> Vector3:
    if text is None:
        return default
    values = tuple(float(value) for value in text.split())
    if len(values) != 3 or any(not math.isfinite(value) for value in values):
        raise ValueError("URDF vector must contain three finite values")
    return values  # type: ignore[return-value]


def _identity() -> Matrix3:
    return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def _matmul(a: Matrix3, b: Matrix3) -> Matrix3:
    return tuple(tuple(sum(a[r][k] * b[k][c] for k in range(3))
                       for c in range(3)) for r in range(3))


def _matvec(a: Matrix3, v: Vector3) -> Vector3:
    return tuple(sum(a[r][k] * v[k] for k in range(3))
                 for r in range(3))  # type: ignore[return-value]


def _transpose(a: Matrix3) -> Matrix3:
    return tuple(tuple(a[r][c] for r in range(3)) for c in range(3))


def _rpy_matrix(roll: float, pitch: float, yaw: float) -> Matrix3:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # URDF fixed-axis roll, pitch, yaw is Rz(yaw) * Ry(pitch) * Rx(roll).
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _axis_rotation(axis: Vector3, angle: float) -> Matrix3:
    x, y, z = axis
    c, s, t = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return (
        (t*x*x + c, t*x*y - s*z, t*x*z + s*y),
        (t*x*y + s*z, t*y*y + c, t*y*z - s*x),
        (t*x*z - s*y, t*y*z + s*x, t*z*z + c),
    )


def _cross(a: Vector3, b: Vector3) -> Vector3:
    return (a[1]*b[2] - a[2]*b[1],
            a[2]*b[0] - a[0]*b[2],
            a[0]*b[1] - a[1]*b[0])


def _rotation_vector(rotation: Matrix3) -> Vector3:
    cosine = max(-1.0, min(1.0,
        (rotation[0][0] + rotation[1][1] + rotation[2][2] - 1.0) / 2.0))
    angle = math.acos(cosine)
    skew = (rotation[2][1] - rotation[1][2],
            rotation[0][2] - rotation[2][0],
            rotation[1][0] - rotation[0][1])
    if angle < 1e-8:
        return tuple(0.5 * value for value in skew)  # type: ignore[return-value]
    if math.pi - angle < 1e-5:
        diagonal = tuple(max(0.0, (rotation[i][i] + 1.0) / 2.0)
                         for i in range(3))
        axis = [math.sqrt(value) for value in diagonal]
        if rotation[0][1] + rotation[1][0] < 0.0:
            axis[1] = -axis[1]
        if rotation[0][2] + rotation[2][0] < 0.0:
            axis[2] = -axis[2]
        norm = math.sqrt(sum(value * value for value in axis))
        if norm < 1e-10:
            raise ValueError("rotation error is numerically ambiguous at pi")
        return tuple(angle * value / norm for value in axis)  # type: ignore[return-value]
    scale = angle / (2.0 * math.sin(angle))
    return tuple(scale * value for value in skew)  # type: ignore[return-value]


def _solve_linear(matrix: list[list[float]], rhs: list[float]) -> list[float]:
    """Solve a small dense system with pivoting and reject a singular matrix."""
    size = len(rhs)
    work = [list(matrix[row]) + [rhs[row]] for row in range(size)]
    for column in range(size):
        pivot = max(range(column, size), key=lambda row: abs(work[row][column]))
        if abs(work[pivot][column]) < 1e-13:
            raise ValueError("IK Jacobian is singular")
        work[column], work[pivot] = work[pivot], work[column]
        divisor = work[column][column]
        work[column] = [value / divisor for value in work[column]]
        for row in range(size):
            if row == column:
                continue
            factor = work[row][column]
            if factor:
                work[row] = [work[row][k] - factor * work[column][k]
                             for k in range(size + 1)]
    result = [work[row][-1] for row in range(size)]
    if any(not math.isfinite(value) for value in result):
        raise ValueError("IK linear solve produced a non-finite update")
    return result


def _condition_inf(matrix: list[list[float]]) -> float:
    size = len(matrix)
    inverse_columns = []
    for column in range(size):
        rhs = [1.0 if row == column else 0.0 for row in range(size)]
        inverse_columns.append(_solve_linear(matrix, rhs))
    inverse = [[inverse_columns[column][row] for column in range(size)]
               for row in range(size)]
    norm = max(sum(abs(value) for value in row) for row in matrix)
    inverse_norm = max(sum(abs(value) for value in row) for row in inverse)
    return norm * inverse_norm


class PiperKinematics:
    """Read the real Piper chain and compute its flange-local tool pose."""

    def __init__(self, urdf_path: str | Path,
                 tool_translation_m: Vector3 = DEFAULT_TOOL_TRANSLATION_M):
        self.urdf_path = Path(urdf_path).expanduser().resolve()
        if not self.urdf_path.is_file():
            raise ValueError(f"PiPER URDF not found: {self.urdf_path}")
        self.tool_translation_m = tuple(float(v) for v in tool_translation_m)
        if (len(self.tool_translation_m) != 3
                or any(not math.isfinite(v) for v in self.tool_translation_m)):
            raise ValueError("tool translation must be three finite metres")
        root = ET.parse(self.urdf_path).getroot()
        if root.tag != "robot" or root.attrib.get("name") != "piper":
            raise ValueError("expected the original Piper URDF, not Piper X")
        parent_by_child: dict[str, ET.Element] = {}
        for element in root.findall("joint"):
            child = element.find("child")
            if child is None:
                continue
            link_name = child.attrib.get("link", "")
            if link_name in parent_by_child:
                raise ValueError(f"URDF link has multiple parent joints: {link_name}")
            parent_by_child[link_name] = element

        reverse_chain: list[ET.Element] = []
        current_link = "link6"
        for _ in range(16):
            if current_link == "base_link":
                break
            element = parent_by_child.get(current_link)
            if element is None:
                raise ValueError(f"URDF has no path from base_link to {current_link}")
            parent = element.find("parent")
            if parent is None:
                raise ValueError("URDF joint is missing its parent link")
            reverse_chain.append(element)
            current_link = parent.attrib.get("link", "")
        else:
            raise ValueError("URDF chain is cyclic or too long")

        parsed: list[_Joint] = []
        for element in reversed(reverse_chain):
            name = element.attrib.get("name", "")
            if element.attrib.get("type") != "revolute":
                raise ValueError(f"expected a revolute arm joint in chain: {name}")
            origin = element.find("origin")
            axis_element = element.find("axis")
            limit = element.find("limit")
            if axis_element is None or limit is None:
                raise ValueError(f"URDF joint lacks axis or limits: {name}")
            xyz = _vec3(origin.attrib.get("xyz") if origin is not None else None,
                         (0.0, 0.0, 0.0))
            rpy = _vec3(origin.attrib.get("rpy") if origin is not None else None,
                         (0.0, 0.0, 0.0))
            axis = _vec3(axis_element.attrib.get("xyz"), (0.0, 0.0, 0.0))
            axis_norm = math.sqrt(sum(value * value for value in axis))
            if axis_norm < 1e-12:
                raise ValueError(f"URDF joint axis has zero length: {name}")
            lower, upper = float(limit.attrib["lower"]), float(limit.attrib["upper"])
            if not math.isfinite(lower + upper) or lower >= upper:
                raise ValueError(f"URDF joint limits are invalid: {name}")
            parsed.append(_Joint(name, xyz, rpy,
                tuple(value / axis_norm for value in axis), lower, upper))
        if tuple(joint.name for joint in parsed) != JOINT_NAMES:
            raise ValueError("URDF chain must be original Piper joint1..joint6")
        self.joints = tuple(parsed)
        self.urdf_limits_rad = tuple((joint.lower_rad, joint.upper_rad)
                                     for joint in self.joints)
        self.maximum_reach_m = sum(
            math.sqrt(sum(component * component for component in joint.xyz_m))
            for joint in self.joints
        ) + math.sqrt(sum(value * value for value in self.tool_translation_m))

    def forward(self, joints_rad: tuple[float, ...] | list[float]) -> Pose:
        if len(joints_rad) != 6 or any(not math.isfinite(float(q)) for q in joints_rad):
            raise ValueError("FK requires six finite joint values in radians")
        position: Vector3 = (0.0, 0.0, 0.0)
        rotation = _identity()
        for joint, value in zip(self.joints, joints_rad):
            origin_rotation = _rpy_matrix(*joint.rpy_rad)
            origin_position = _matvec(rotation, joint.xyz_m)
            position = tuple(position[i] + origin_position[i]
                             for i in range(3))  # type: ignore[assignment]
            rotation = _matmul(rotation, origin_rotation)
            rotation = _matmul(rotation, _axis_rotation(joint.axis, float(value)))
        tool_offset = _matvec(rotation, self.tool_translation_m)
        tcp = tuple(position[i] + tool_offset[i] for i in range(3))
        return Pose(tcp, rotation)  # type: ignore[arg-type]

    def _jacobian(self, joints_rad: tuple[float, ...]) -> list[list[float]]:
        position: Vector3 = (0.0, 0.0, 0.0)
        rotation = _identity()
        origins: list[Vector3] = []
        axes_base: list[Vector3] = []
        for joint, value in zip(self.joints, joints_rad):
            joint_origin = tuple(position[i] + _matvec(rotation, joint.xyz_m)[i]
                                 for i in range(3))
            origin_rotation = _matmul(rotation, _rpy_matrix(*joint.rpy_rad))
            origins.append(joint_origin)  # type: ignore[arg-type]
            axes_base.append(_matvec(origin_rotation, joint.axis))
            position = joint_origin  # type: ignore[assignment]
            rotation = _matmul(origin_rotation,
                               _axis_rotation(joint.axis, value))
        tcp_offset = _matvec(rotation, self.tool_translation_m)
        tcp = tuple(position[i] + tcp_offset[i] for i in range(3))
        jacobian = [[0.0] * 6 for _ in range(6)]
        for column, (origin, axis) in enumerate(zip(origins, axes_base)):
            linear = _cross(axis, tuple(tcp[i] - origin[i] for i in range(3)))
            for row in range(3):
                jacobian[row][column] = linear[row]
                jacobian[row + 3][column] = (
                    ORIENTATION_METERS_PER_RADIAN * axis[row]
                )
        return jacobian

    def inverse(self, target: Pose, seed_rad: tuple[float, ...], *,
                joint_limits_rad: tuple[tuple[float, float], ...] | None = None,
                max_iterations: int = 80,
                max_condition: float = MAX_JACOBIAN_CONDITION,
                allow_damped_singularity: bool = False,
                position_tolerance_m: float = 2e-5,
                orientation_tolerance_rad: float = 2e-4,
                time_budget_s: float | None = None,
                minimum_joint_change_rad: float = 0.,
                polish_iterations: int = 0) -> tuple[float, ...]:
        """Damped least-squares IK seeded from the latest continuous solution."""
        limits = joint_limits_rad or self.urdf_limits_rad
        if (not math.isfinite(position_tolerance_m+orientation_tolerance_rad)
                or position_tolerance_m <= 0 or orientation_tolerance_rad <= 0):
            raise ValueError('IK tolerances must be finite and positive')
        if not math.isfinite(minimum_joint_change_rad) or minimum_joint_change_rad < 0:
            raise ValueError('minimum joint progress must be finite and nonnegative')
        if type(polish_iterations) is not int or not 0 <= polish_iterations <= 4:
            raise ValueError('IK polish iterations must be an integer in 0..4')
        import time
        deadline = None if time_budget_s is None else time.perf_counter()+max(0.,time_budget_s)
        if len(seed_rad) != 6 or len(limits) != 6:
            raise ValueError("IK requires six seed joints and six joint limits")
        if any(not math.isfinite(float(v)) for v in seed_rad):
            raise ValueError("IK seed must be finite")
        q = [float(value) for value in seed_rad]
        self.last_ik_diagnostics = {}
        for value, (low, high) in zip(q, limits):
            if not low <= value <= high:
                raise ValueError("IK seed is outside the effective joint limits")
        if (len(target.position_m) != 3
                or any(not math.isfinite(value) for value in target.position_m)):
            raise ValueError("IK target position must contain three finite metres")
        base_distance = math.sqrt(sum(value * value for value in target.position_m))
        if base_distance > self.maximum_reach_m + 1e-6:
            raise ValueError("IK target is unreachable outside the URDF chain reach")

        damping = 0.01
        best = None
        best_score = math.inf
        first_accepted = None
        best_diagnostics = None
        for iteration in range(max_iterations):
            if deadline is not None and time.perf_counter() >= deadline:
                if best is not None:
                    self.last_ik_diagnostics = best_diagnostics
                    return best
                raise ValueError('IK_TIMEOUT: numerical solve time budget exhausted')
            actual = self.forward(q)
            position_error = tuple(target.position_m[i] - actual.position_m[i]
                                   for i in range(3))
            orientation_error = _rotation_vector(
                _matmul(target.rotation, _transpose(actual.rotation)))
            error = list(position_error) + [
                ORIENTATION_METERS_PER_RADIAN * value
                for value in orientation_error
            ]
            position_norm = math.sqrt(sum(value * value for value in position_error))
            orientation_norm = math.sqrt(sum(value * value
                                             for value in orientation_error))
            self.last_ik_diagnostics.update(iterations=iteration+1,
                position_residual_m=position_norm,orientation_residual_rad=orientation_norm)
            if (position_norm <= position_tolerance_m and orientation_norm <= orientation_tolerance_rad
                    and max(abs(a-b) for a,b in zip(q,seed_rad)) >= minimum_joint_change_rad):
                if not polish_iterations:
                    return tuple(q)
                score = (position_norm/position_tolerance_m)**2 + (orientation_norm/orientation_tolerance_rad)**2
                if score < best_score:
                    best, best_score = tuple(q), score
                    best_diagnostics = dict(self.last_ik_diagnostics)
                if first_accepted is None:
                    first_accepted = iteration
            if first_accepted is not None and iteration >= first_accepted+polish_iterations:
                self.last_ik_diagnostics = best_diagnostics
                return best

            jacobian = self._jacobian(tuple(q))
            if allow_damped_singularity:
                # SVD DLS avoids forming J.T @ J (which squares conditioning)
                # and six inverse-column solves per iteration. Regularization
                # varies continuously with the weakest singular direction.
                u, singular, vt = np.linalg.svd(np.asarray(jacobian), full_matrices=False)
                proximity = max(0., 1.-float(singular[-1])/.01)
                damping = .01 + .01*proximity*proximity
                self.last_ik_diagnostics.update(damping=damping,
                    minimum_singular_value=float(singular[-1]),
                    jacobian_condition=None if singular[-1] <= 1e-15 else float(singular[0]/singular[-1]))
                gain = singular / (singular*singular+damping*damping)
                delta = (vt.T @ (gain*(u.T @ np.asarray(error)))).tolist()
                largest = max(abs(value) for value in delta)
                if largest > .2:
                    delta = [v*.2/largest for v in delta]
                for index,(low,high) in enumerate(limits):
                    q[index] = max(low,min(high,q[index]+delta[index]))
                continue
            try:
                condition = _condition_inf(jacobian)
            except ValueError:
                condition = math.inf
            if not math.isfinite(condition) or condition > max_condition:
                if not allow_damped_singularity:
                    raise ValueError(f"IK rejected near-singular pose (condition={condition:.3g})")
                # Bounded teleop may try a regularized solution; convergence
                # tolerances and the caller's feedback step limits still apply.
                # The default IK API retains its strict singularity rejection.
            damping = 0.01  # Preserve the strict, non-teleop API behavior.
            transpose = [[jacobian[row][column]
                          for row in range(6)] for column in range(6)]
            normal = [[sum(transpose[row][k] * jacobian[k][column]
                           for k in range(6))
                       + (damping * damping if row == column else 0.0)
                       for column in range(6)] for row in range(6)]
            rhs = [sum(transpose[row][k] * error[k] for k in range(6))
                   for row in range(6)]
            delta = _solve_linear(normal, rhs)
            largest = max(abs(value) for value in delta)
            if largest > 0.2:
                scale = 0.2 / largest
                delta = [value * scale for value in delta]
            for index, (low, high) in enumerate(limits):
                q[index] = max(low, min(high, q[index] + delta[index]))
        if best is not None:
            self.last_ik_diagnostics = best_diagnostics
            return best
        raise ValueError("IK_NOT_CONVERGED: iteration budget exhausted; reachability not established")


def apply_pose_delta(pose: Pose, translation_m: Vector3,
                     rotation_vector_rad: Vector3, *, frame: str) -> Pose:
    """Apply a six-dimensional incremental displacement in base or local axes."""
    if frame not in ("base", "local"):
        raise ValueError("Cartesian coordinate frame must be 'base' or 'local'")
    if any(not math.isfinite(value)
           for value in (*translation_m, *rotation_vector_rad)):
        raise ValueError("Cartesian increments must be finite")
    translation_base = (
        translation_m if frame == "base"
        else _matvec(pose.rotation, translation_m)
    )
    position = tuple(pose.position_m[i] + translation_base[i]
                     for i in range(3))
    magnitude = math.sqrt(sum(value * value for value in rotation_vector_rad))
    if magnitude <= 1e-12:
        return Pose(position, pose.rotation)
    axis = tuple(value / magnitude for value in rotation_vector_rad)
    delta_rotation = _axis_rotation(axis, magnitude)
    rotation = (
        _matmul(delta_rotation, pose.rotation) if frame == "base"
        else _matmul(pose.rotation, delta_rotation)
    )
    return Pose(position, rotation)  # type: ignore[arg-type]


def resolve_default_piper_urdf() -> Path:
    """Find the exact project Piper URDF without selecting the Piper X model."""
    import os

    configured = os.environ.get("PI05_PIPER_URDF") or os.environ.get("PI05_QUEST_URDF")
    candidates = []
    if configured:
        candidates.append(Path(configured).expanduser())
    candidates.extend((
        Path(__file__).resolve().parents[4]
        / "quest_capture" / "upstream" / "agx_arm_urdf" / "piper"
        / "urdf" / "piper_with_gripper_description.urdf",
        Path.cwd() / "quest_capture" / "upstream" / "agx_arm_urdf" / "piper"
        / "urdf" / "piper_with_gripper_description.urdf",
    ))
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    raise ValueError(
        "original PiPER URDF unavailable; set PI05_PIPER_URDF to "
        "quest_capture/upstream/agx_arm_urdf/piper/urdf/"
        "piper_with_gripper_description.urdf"
    )
