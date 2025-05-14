#!/usr/bin/env python3

import time
import rclpy
import math
import threading

import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, PoseStamped, Twist, TransformStamped, Vector3Stamped
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import TransformListener, Buffer
from tf2_ros import StaticTransformBroadcaster, TransformStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.executors import MultiThreadedExecutor


'''
Warehouse project - Nav2 Navigation with API
Move robot to shelf and then to the shipping_position carrying the shelf.
'''

class WarehouseNavigator(rclpy.node.Node):
    def __init__(self):
        super().__init__('warehouse_navigator')
        self.navigator = BasicNavigator()
        
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/diffbot_base_controller/cmd_vel_unstamped', 10)
        self.attach_pub = self.create_publisher(String, '/elevator_up', 1)
        self.detach_pub = self.create_publisher(String, '/elevator_down', 1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        
        # Waypoints [x, y, yaw]
        self.task_route = [
            [0.014, 0.029, 0.0],      # Initial pose
            [5.606, -0.022, -1.574],  # Loading
            [2.411, 1.122, 1.574],    # Shipping
        ]

        # Current state variables
        self.robot_odom = None
        self.laser_ranges = []
        self.laser_intensities = []
        self.angle_increment = 0
        self.angle_min = 0
        self.cart_pose = PoseStamped()

        # Prepare initial pose
        self.initial_pose = self.create_pose(*self.task_route[0])
        self.get_logger().info("Warehouse Navigator app started.")

    def create_pose(self, x, y, yaw):
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
        self.get_logger().info("Setting initial pose...")
        self.navigator.setInitialPose(self.initial_pose)

    def go_to_pose(self, pose, waypoint_name=""):
        self.navigator.goToPose(pose)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(f"Estimated time remaining: {feedback.estimated_time_remaining.sec} sec")

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f"Reached {waypoint_name} destination successfully.")
        elif result == TaskResult.CANCELED:
            self.get_logger().info("Navigation canceled.")
            exit(1)
        elif result == TaskResult.FAILED:
            self.get_logger().info("Navigation failed.")
            exit(1)

    def scan_callback(self, msg):
        if not msg.ranges or not msg.intensities:
            self.get_logger().warn("Missing data for laser scan.")
            return
        self.laser_ranges = msg.ranges
        self.laser_intensities = msg.intensities
        self.angle_increment = msg.angle_increment
        self.angle_min = msg.angle_min

    def odom_callback(self, msg):
        self.robot_odom = msg

    def approach_cart(self):
        leg_data = []
        nlegs = 0
        leg_flag = False
        leg_start = 0
        for i, intensity in enumerate(self.laser_intensities):
            if intensity > 100.0:
                if not leg_flag:
                    leg_flag = True
                    leg_start = i
            else:
                if leg_flag:
                    leg_flag = False
                    leg_center = (leg_start + i) // 2
                    nlegs += 1
                    leg_data.append((leg_center, self.laser_ranges[leg_center]))

        if nlegs < 2:
            self.get_logger().warn("Less than 2 reflectors found.")
            return

        a = leg_data[0][1]
        b = leg_data[1][1]
        theta = 2 * math.pi * abs(leg_data[0][0] - leg_data[1][0]) / len(self.laser_ranges)
        c = math.sqrt(a**2 + b**2 - 2 * a * b * math.cos(theta))
        d = 0.5 * math.sqrt(2 * (a**2 + b**2) - c**2)
        alpha = math.asin(a * math.sin(theta) / c)
        beta = math.asin((c / 2) * math.sin(alpha) / d)
        mid_x = d * math.sin(alpha + beta)
        mid_y = d * math.cos(alpha + beta)

        self.broadcast_cart_frame(mid_x, mid_y)

        # Move the robot to the attaching position
        self.move_to_attach()

    def broadcast_cart_frame(self, x, y):
        """Broadcast the cart frame transform from laser frame to map frame."""
        try:
            t = self.tf_buffer.lookup_transform('map', 'robot_front_laser_base_link', rclpy.time.Time())
            laser_point = PointStamped()
            laser_point.header.frame_id = 'robot_front_laser_base_link'
            laser_point.header.stamp = t.header.stamp
            laser_point.point.x = x
            laser_point.point.y = y

            odom_point = tf2_geometry_msgs.do_transform_point(laser_point, t)

            cart_transform = TransformStamped()
            cart_transform.header.stamp = self.get_clock().now().to_msg()
            cart_transform.header.frame_id = 'map'
            cart_transform.child_frame_id = 'cart_frame'
            cart_transform.transform.translation.x = odom_point.point.x
            cart_transform.transform.translation.y = odom_point.point.y
            cart_transform.transform.translation.z = 0.0

            q = quaternion_from_euler(0, 0, -1.574)
            cart_transform.transform.rotation.x = q[0]
            cart_transform.transform.rotation.y = q[1]
            cart_transform.transform.rotation.z = q[2]
            cart_transform.transform.rotation.w = q[3]

            self.tf_broadcaster.sendTransform(cart_transform)

        except Exception as e:
            self.get_logger().error(f"Error broadcasting cart frame: {e}")

    def move_to_attach(self):
        """Move robot directly to cart loading position."""

        self.get_logger().info("Moving to cart loading position...")

        rate = self.create_rate(10)
        while rclpy.ok():

            t = TransformStamped()
            # Wait until the transform is available
            while True:
                try:
                    t = self.tf_buffer.lookup_transform('robot_base_link', 'cart_frame', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1))
                    break
                except tf2_ros.TransformException:
                    self.get_logger().warn("Waiting for 'cart_frame' to be available in TF tree...")
                    time.sleep(1)  # Wait and retry
            
            dx = t.transform.translation.x + 0.5
            dy = t.transform.translation.y

            error_distance = math.sqrt(dx**2 + dy**2)
            error_yaw = math.atan2(dy, dx)
            error_yaw = (error_yaw + math.pi) % (2 * math.pi) - math.pi # Normalize angle to [-pi, pi]

            vel_msg = Twist()

            self.get_logger().info(f"Goal dist: {error_distance}")
            if error_distance > 0.1:
                vel_msg.linear.x = min(1.0 * error_distance, 0.3)
                vel_msg.angular.z = -0.5 * error_yaw
            else:
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = 0.0
                self.get_logger().info("Approach completed.")
                return

            self.cmd_vel_pub.publish(vel_msg)
            rate.sleep()


    def perform_tasks(self):
        for i, (x, y, yaw) in enumerate(self.task_route[1:], start=1):
            pose = self.create_pose(x, y, yaw)
            self.go_to_pose(pose, f"Waypoint {i}")

            if i == 1:
                self.approach_cart()
                self.get_logger().info("Lifting the shelf")
                self.attach_pub.publish(String())
                self.get_logger().info("Shelf attached. Moving to Shipping position")
            else:
                self.get_logger().info("At Shipping position. Detaching the shelf")
                self.detach_pub.publish(String())

    def return_to_start(self):
        self.get_logger().info("Returning to initial pose...")
        self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.go_to_pose(self.initial_pose, "Initial Pose")

    def shutdown(self):
        self.get_logger().info("All tasks completed. Shutting down.")
        rclpy.shutdown()
        exit(0)


if __name__ == '__main__':
    rclpy.init()

    robot_node = WarehouseNavigator()
    robot_node.setup()

    executor = MultiThreadedExecutor()
    executor.add_node(robot_node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        while rclpy.ok():
            robot_node.perform_tasks()
            robot_node.return_to_start()
            robot_node.shutdown()
            time.sleep(1)

    except KeyboardInterrupt:
        print("Shutting down gracefully.")

    executor.shutdown()

