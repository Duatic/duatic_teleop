#!/usr/bin/env python3


import time

import threading


import rclpy

from rclpy.node import Node


from sensor_msgs.msg import JointState

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint



class ABMover(Node):

    def __init__(self):

        super().__init__("ab_mover")


        self.joint_names = None

        self.current_positions = None


        self.joint_state_sub = self.create_subscription(

            JointState,

            "/joint_states",

            self.joint_state_callback,

            10,

        )


        self.traj_pub = self.create_publisher(

            JointTrajectory,

            "/joint_trajectory_controller/joint_trajectory",

            10,

        )


        self.get_logger().info("Node started, move the robot to the start position.")

        self.get_logger().info("Press ENTER when the robot is at the desired start position.")


        self.setup_thread = threading.Thread(target=self.interactive_setup)

        self.setup_thread.daemon = True

        self.setup_thread.start()


    def joint_state_callback(self, msg: JointState):

        self.joint_names = list(msg.name)

        self.current_positions = list(msg.position)


    def wait_for_joint_state(self):

        while rclpy.ok() and (

            self.joint_names is None or self.current_positions is None

        ):

            time.sleep(0.1)


    def get_current_pose(self):

        self.wait_for_joint_state()

        return list(self.current_positions)


    def publish_position(self, positions, duration_sec):

        msg = JointTrajectory()


        msg.header.stamp = self.get_clock().now().to_msg()

        msg.joint_names = self.joint_names


        point = JointTrajectoryPoint()

        point.positions = list(positions)

        point.velocities = [0.0] * len(positions)


        sec = int(duration_sec)

        nanosec = int((duration_sec - sec) * 1e9)


        point.time_from_start.sec = sec

        point.time_from_start.nanosec = nanosec


        msg.points.append(point)


        self.get_logger().info(

            f"Sending trajectory command with duration {duration_sec:.2f}s"

        )


        self.get_logger().info(f"publishing msg: {msg}")


        self.traj_pub.publish(msg)


    def interactive_setup(self):

        input()


        start_position = self.get_current_pose()


        self.get_logger().info("Start position recorded.")


        waypoints = []


        while True:

            print()

            user_input = input(

                "Move the robot to the next waypoint and enter the travel time in seconds "

                "(e.g. 4.5). Press ENTER for default 5.0s or enter 'q' to finish: "

            ).strip()


            if user_input.lower() == "q":

                break


            duration = 5.0


            if user_input:

                try:

                    duration = float(user_input)

                except ValueError:

                    self.get_logger().info(

                        "Invalid number entered. Using default duration of 5.0s."

                    )


            input(

                "Press ENTER after the robot has been moved to the desired waypoint..."

            )


            waypoint = self.get_current_pose()


            waypoints.append(

                {

                    "positions": waypoint,

                    "duration": duration,

                }

            )


            self.get_logger().info(

                f"Waypoint {len(waypoints)} recorded with duration {duration:.2f}s"

            )


        if len(waypoints) == 0:

            self.get_logger().info(

                "No waypoints recorded. Exiting waypoint loop."

            )

            return


        self.get_logger().info(

            f"Setup complete. Recorded {len(waypoints)} waypoint(s)."

        )


        self.get_logger().info(

            "Waiting 5 seconds before starting motion..."

        )

        time.sleep(5.0)


        self.get_logger().info(

            "Driving to start position."

        )


        self.publish_position(start_position, 5.0)


        time.sleep(5.0)


        while rclpy.ok():

            for i, waypoint in enumerate(waypoints):

                self.get_logger().info(

                    f"Driving to waypoint {i + 1}/{len(waypoints)} "

                    f"(duration {waypoint['duration']:.2f}s)"

                )


                self.publish_position(

                    waypoint["positions"],

                    waypoint["duration"],

                )


                time.sleep(waypoint["duration"])


            self.get_logger().info(

                "Last waypoint reached. Waiting 3 seconds."

            )


            time.sleep(3.0)


            self.get_logger().info(

                "Returning to start position."

            )


            self.publish_position(start_position, 5.0)


            time.sleep(5.0)



def main(args=None):

    rclpy.init(args=args)


    node = ABMover()


    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass


    node.destroy_node()

    rclpy.shutdown()



if __name__ == "__main__":

    main()