#!/usr/bin/env python3

import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler

'''
Warehouse project
Move robot to shelf and to shipping_position in order while doing specified tasks.
'''

class WarehouseNavigator:
    def __init__(self):
        rclpy.init()
        self.navigator = BasicNavigator()

        # Waypoints [x, y, yaw]
        self.task_route = [
            [0.014, 0.029, 0.0],      # Initial pose
            [5.606, -0.022, -1.574],  # Loading
            [2.534, 1.246, 1.574],    # Shipping
        ]

        # Prepare initial pose
        self.initial_pose = self._create_pose(*self.task_route[0])

    def _create_pose(self, x, y, yaw):
        """Create a PoseStamped from x, y, yaw."""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()

        quat = quaternion_from_euler(0, 0, yaw)
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        return pose

    def setup(self):
        print("Setting initial pose...")
        self.navigator.setInitialPose(self.initial_pose)
        self.navigator.waitUntilNav2Active()

    def go_to_pose(self, pose, waypoint_name=""):
        print(f"Navigating to: {waypoint_name if waypoint_name else pose.pose.position}")
        self.navigator.goToPose(pose)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                print(f"Moving... Estimated time remaining: {feedback.estimated_time_remaining.sec} sec")

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print("Reached destination successfully.")
        elif result == TaskResult.CANCELED:
            print("Navigation canceled.")
            exit(1)
        elif result == TaskResult.FAILED:
            print("Navigation failed.")
            exit(1)

    def perform_tasks(self):
        for i, (x, y, yaw) in enumerate(self.task_route[1:], start=1):  # skip initial
            pose = self._create_pose(x, y, yaw)
            self.go_to_pose(pose, f"Waypoint {i}")
            print("Performing task... Waiting 2 seconds.")
            time.sleep(2)

    def return_to_start(self):
        print("Returning to initial pose...")
        self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.go_to_pose(self.initial_pose, "Initial Pose")

    def shutdown(self):
        print("Task complete. Shutting down.")
        rclpy.shutdown()
        exit(0)


def main():
    robot = WarehouseNavigator()
    robot.setup()
    robot.perform_tasks()
    robot.return_to_start()
    robot.shutdown()


if __name__ == '__main__':
    main()
