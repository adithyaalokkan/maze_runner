#!usr/bin/env python3


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
import tf_transformations


class Commander(Node):

    def __init__(self):
        super().__init__("commander_node")
        self.navigate()

    def navigate(self):
        goals = []
        navigator = BasicNavigator()

        initial_pose = PoseStamped()
        initial_pose.header.frame_id = "map"
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 3.5
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.orientation.z = 0.0
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, 1.57)
        initial_pose.pose.orientation.x = q_x
        initial_pose.pose.orientation.y = q_y
        initial_pose.pose.orientation.z = q_z
        initial_pose.pose.orientation.w = q_w
        navigator.setInitialPose(initial_pose)

        navigator.waitUntilNav2Active()

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = -3.5
        goal_pose.pose.position.y = 0.0
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, 3.14)
        initial_pose.pose.orientation.x = q_x
        initial_pose.pose.orientation.y = q_y
        initial_pose.pose.orientation.z = q_z
        initial_pose.pose.orientation.w = q_w
        goals.append(goal_pose)

        navigator.goThroughPoses(goals)
        while not navigator.isTaskComplete():
            pass

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print("Goal succeeded!")
        elif result == TaskResult.CANCELED:
            print("Goal was canceled!")
        elif result == TaskResult.FAILED:
            print("Goal failed!")
        else:
            print("Goal has an invalid return status!")

        navigator.lifecycleShutdown()

        return


def main(args=None):
    rclpy.init()

    print("Starting Commander Node...")
    commander_node = Commander()
    commander_node.destroy_node()
    print("Destroyed Commander Node!")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
