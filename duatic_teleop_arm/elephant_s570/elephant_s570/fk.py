"""Forward kinematics for the Elephant Robotics myController S570.

Parses the S570 URDF and provides FK computation for both arms using
pure numpy — no external robotics library required.

The S570 has two 7-DOF arms:
  Left arm:  joint1–joint7   → end-effector at link7
  Right arm: joint8–joint14  → end-effector at link14
"""

import xml.etree.ElementTree as ET
import numpy as np
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

# Default URDF path (installed via ament into share/)
_DEFAULT_URDF = Path(get_package_share_directory("elephant_s570")) / "urdf" / "s570.urdf"


def _rotation_x(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array(
        [
            [1, 0, 0, 0],
            [0, c, -s, 0],
            [0, s, c, 0],
            [0, 0, 0, 1],
        ]
    )


def _rotation_y(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array(
        [
            [c, 0, s, 0],
            [0, 1, 0, 0],
            [-s, 0, c, 0],
            [0, 0, 0, 1],
        ]
    )


def _rotation_z(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array(
        [
            [c, -s, 0, 0],
            [s, c, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ]
    )


def _transform_from_origin(xyz: list[float], rpy: list[float]) -> np.ndarray:
    """Build a 4x4 homogeneous transform from URDF origin xyz + rpy."""
    T = np.eye(4)
    T[0, 3], T[1, 3], T[2, 3] = xyz
    # URDF RPY: rotate around fixed axes X, Y, Z (extrinsic)
    R = _rotation_z(rpy[2]) @ _rotation_y(rpy[1]) @ _rotation_x(rpy[0])
    T[:3, :3] = R[:3, :3]
    return T


def _rotation_about_axis(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues rotation as a 4x4 homogeneous transform."""
    ax = axis / np.linalg.norm(axis)
    c, s = np.cos(angle), np.sin(angle)
    t = 1.0 - c
    x, y, z = ax
    R = np.array(
        [
            [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
        ]
    )
    T = np.eye(4)
    T[:3, :3] = R
    return T


def _rotation_matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
    """Convert 3x3 rotation matrix to quaternion [w, x, y, z]."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return np.array([w, x, y, z])


class _JointDef:
    """Parsed joint definition from URDF."""

    def __init__(self, name: str, axis: np.ndarray, origin: np.ndarray):
        self.name = name
        self.axis = axis
        self.origin = origin  # 4x4 homogeneous transform


class S570FK:
    """Forward kinematics for one or both S570 arms.

    Usage:
        fk = S570FK()
        pos, quat = fk.compute("left", joint_angles_rad)
    """

    def __init__(self, urdf_path: str | Path | None = None):
        urdf_path = Path(urdf_path) if urdf_path else _DEFAULT_URDF
        tree = ET.parse(urdf_path)
        root = tree.getroot()

        # Parse all joints
        all_joints: dict[str, _JointDef] = {}
        for joint_elem in root.findall("joint"):
            jtype = joint_elem.get("type")
            if jtype != "revolute":
                continue

            name = joint_elem.get("name")
            axis_elem = joint_elem.find("axis")
            axis_str = axis_elem.get("xyz").split()
            axis = np.array([float(v) for v in axis_str])

            origin_elem = joint_elem.find("origin")
            xyz = [float(v) for v in origin_elem.get("xyz").split()]
            rpy_str = origin_elem.get("rpy", "0 0 0").split()
            rpy = [float(v) for v in rpy_str]
            origin_T = _transform_from_origin(xyz, rpy)

            all_joints[name] = _JointDef(name, axis, origin_T)

        # Build kinematic chains
        left_joint_names = [f"joint{i}" for i in range(1, 8)]
        right_joint_names = [f"joint{i}" for i in range(8, 15)]

        self._left_chain = [all_joints[n] for n in left_joint_names]
        self._right_chain = [all_joints[n] for n in right_joint_names]

    def compute(self, side: str, joint_angles_rad: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Compute FK for one arm.

        Args:
            side: "left" or "right"
            joint_angles_rad: Array of 7 joint angles in radians.

        Returns:
            (position[3], quaternion_wxyz[4]) of the end-effector.
        """
        chain = self._left_chain if side == "left" else self._right_chain
        assert len(joint_angles_rad) >= len(
            chain
        ), f"Expected {len(chain)} joint angles, got {len(joint_angles_rad)}"

        T = np.eye(4)
        for i, joint in enumerate(chain):
            T = T @ joint.origin @ _rotation_about_axis(joint.axis, joint_angles_rad[i])

        pos = T[:3, 3]
        quat = _rotation_matrix_to_quaternion(T[:3, :3])
        return pos, quat

    def compute_relative(
        self,
        side: str,
        home_angles_rad: np.ndarray,
        current_angles_rad: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Compute the relative end-effector displacement from home to current.

        All deltas are expressed in the **base frame** so that "up" on the S570
        maps to "up" on the robot, regardless of the EE orientation.

        Returns:
            (delta_position[3], delta_quaternion_wxyz[4])
        """
        chain = self._left_chain if side == "left" else self._right_chain

        T_home = np.eye(4)
        for i, joint in enumerate(chain):
            T_home = T_home @ joint.origin @ _rotation_about_axis(joint.axis, home_angles_rad[i])

        T_current = np.eye(4)
        for i, joint in enumerate(chain):
            T_current = (
                T_current @ joint.origin @ _rotation_about_axis(joint.axis, current_angles_rad[i])
            )

        # Position delta in base frame (simple subtraction — already in base frame)
        delta_pos = T_current[:3, 3] - T_home[:3, 3]

        # Orientation delta in base frame: R_delta = R_current @ R_home^T
        R_delta = T_current[:3, :3] @ T_home[:3, :3].T
        delta_quat = _rotation_matrix_to_quaternion(R_delta)
        return delta_pos, delta_quat
