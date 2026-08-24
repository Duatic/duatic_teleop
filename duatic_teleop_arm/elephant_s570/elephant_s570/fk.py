# Copyright 2026 Duatic AG
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that
# the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions, and
#    the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions, and
#    the following disclaimer in the documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or
#    promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Forward kinematics for the Elephant Robotics myController S570.

Parses the S570 URDF and provides FK computation for both arms using
numpy and scipy — no external robotics library required.

The S570 has two 7-DOF arms:
  Left arm:  joint1–joint7   → end-effector at link7
  Right arm: joint8–joint14  → end-effector at link14
"""

import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

# Default URDF path (installed via ament into share/)
_DEFAULT_URDF = Path(get_package_share_directory("elephant_s570")) / "urdf" / "s570.urdf"


def _transform_from_origin(xyz: list[float], rpy: list[float]) -> np.ndarray:
    """Build a 4x4 homogeneous transform from URDF origin xyz + rpy."""
    T = np.eye(4)
    T[0, 3], T[1, 3], T[2, 3] = xyz
    # URDF RPY: rotate around fixed axes X, Y, Z (extrinsic) — scipy's lowercase 'xyz' is
    # the same extrinsic convention.
    T[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    return T


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
            (position[3], quaternion [x, y, z, w][4]) of the end-effector.
        """
        chain = self._left_chain if side == "left" else self._right_chain
        assert len(joint_angles_rad) >= len(
            chain
        ), f"Expected {len(chain)} joint angles, got {len(joint_angles_rad)}"

        T = np.eye(4)
        for i, joint in enumerate(chain):
            joint_rotation = np.eye(4)
            axis = joint.axis / np.linalg.norm(joint.axis)
            joint_rotation[:3, :3] = Rotation.from_rotvec(axis * joint_angles_rad[i]).as_matrix()
            T = T @ joint.origin @ joint_rotation

        pos = T[:3, 3]
        quat = Rotation.from_matrix(T[:3, :3]).as_quat()
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
            (delta_position[3], delta_quaternion [x, y, z, w][4])
        """
        chain = self._left_chain if side == "left" else self._right_chain

        T_home = np.eye(4)
        for i, joint in enumerate(chain):
            joint_rotation = np.eye(4)
            axis = joint.axis / np.linalg.norm(joint.axis)
            joint_rotation[:3, :3] = Rotation.from_rotvec(axis * home_angles_rad[i]).as_matrix()
            T_home = T_home @ joint.origin @ joint_rotation

        T_current = np.eye(4)
        for i, joint in enumerate(chain):
            joint_rotation = np.eye(4)
            axis = joint.axis / np.linalg.norm(joint.axis)
            joint_rotation[:3, :3] = Rotation.from_rotvec(axis * current_angles_rad[i]).as_matrix()
            T_current = T_current @ joint.origin @ joint_rotation

        # Position delta in base frame (simple subtraction — already in base frame)
        delta_pos = T_current[:3, 3] - T_home[:3, 3]

        # Orientation delta in base frame: R_delta = R_current @ R_home^T
        R_delta = T_current[:3, :3] @ T_home[:3, :3].T
        delta_quat = Rotation.from_matrix(R_delta).as_quat()
        return delta_pos, delta_quat
