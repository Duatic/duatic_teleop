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

"""Tests for the duatic_interactive_marker node."""

import rclpy

from duatic_teleop_rviz.duatic_interactive_marker import DuaticInteractiveMarkerNode

README_ROS_ARGS = [
    "--ros-args",
    "-p",
    "pose_topics:=['/cartesian_pose_controller/flange/pose']",
    "-p",
    "target_topics:=['/cartesian_pose_controller/flange/target']",
]


def test_readme_usage_example_instantiates_without_error():
    """Instantiate the node with the README's Usage example --ros-args and check it comes up."""
    rclpy.init(args=README_ROS_ARGS)
    try:
        node = DuaticInteractiveMarkerNode()
        try:
            # give the node a tick to make sure nothing blows up once it's spinning
            rclpy.spin_once(node, timeout_sec=0.1)

            assert len(node.targets) == 1
            target = node.targets[0]
            assert target.index == 0
            assert target.pose_topic == "/cartesian_pose_controller/flange/pose"
            assert target.target_topic == "/cartesian_pose_controller/flange/target"
            assert not target.is_tf
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()
