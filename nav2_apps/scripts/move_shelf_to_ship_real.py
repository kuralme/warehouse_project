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
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

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
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.attach_pub = self.create_publisher(String, '/elevator_up', 1)
        self.detach_pub = self.create_publisher(String, '/elevator_down', 1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = StaticTransformBroadcaster(self)
         
        # Waypoints [x, y, yaw]
        self.task_route = [
            [2.000, -1.912, -1.855],  # Initial pose
            [0.164, -5.924,  2.924],  # Loading
            [2.761, -4.147, -0.284],  # Shipping
        ]

        # Current state variables
        self.robot_odom = None
        self.laser_ranges = []
        self.laser_intensities = []
        self.angle_increment = 0.0
        self.angle_min = 0.0
        self.cart_pose = PoseStamped()

        # Robot footprint dicts
        self.footprint_normal = ({
            "robot_radius": 0.3,
            "inflation_layer.inflation_radius": 0.32,
            "footprint": []
        })
        self.footprint_w_cart = ({
            "robot_radius": 0.0,
            "inflation_layer.inflation_radius": 0.45,
            "footprint": [[0.46, 0.45], [0.46, -0.45], [-0.46, -0.45], [-0.46, 0.45]]
        })

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
                self.get_logger().info(f"{waypoint_name} ETA: {feedback.estimated_time_remaining.sec} sec")

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
        leg_data = self.extract_leg_centers()

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

        # Move the robot to the cart attaching position
        self.move_to_attach()

    def extract_leg_centers(self, intensity_threshold=1500.0, angle_window=math.pi/4, max_range=1.0):
        center_index = 540
        index_range = int(angle_window / self.angle_increment)
        start_index = max(0, center_index - index_range)
        end_index = min(len(self.laser_intensities), center_index + index_range)

        left_candidates = []
        right_candidates = []

        for i in range(start_index, end_index):
            r = self.laser_ranges[i]
            intensity = self.laser_intensities[i]

            if r < max_range and intensity > intensity_threshold:
                if i < center_index:
                    left_candidates.append((i, r, intensity))
                elif i > center_index:
                    right_candidates.append((i, r, intensity))

        if not len(left_candidates) or not len(right_candidates):
            self.get_logger().warn("Not enough valid reflectors found.")
            return []

        # Sort by closeness to center index on each side
        left = sorted(left_candidates, key=lambda x: abs(x[0] - center_index))[0]
        right = sorted(right_candidates, key=lambda x: abs(x[0] - center_index))[0]

        leg_data = [(left[0], left[1]), (right[0], right[1])]
        for i, r in leg_data:
            self.get_logger().info(f"Leg found: Index={i}, Range={r:.2f}")
        return leg_data

    def broadcast_cart_frame(self, x, y):
        """Broadcast the cart frame transform from laser frame to map frame."""
        try:
            t = self.tf_buffer.lookup_transform('map', 'robot_front_laser_base_link', rclpy.time.Time())
            laser_point = PointStamped()
            laser_point.header.frame_id = 'robot_front_laser_base_link'
            laser_point.header.stamp = t.header.stamp
            laser_point.point.x = x
            laser_point.point.y = y

            map_point = tf2_geometry_msgs.do_transform_point(laser_point, t)

            cart_transform = TransformStamped()
            cart_transform.header.stamp = self.get_clock().now().to_msg()
            cart_transform.header.frame_id = 'map'
            cart_transform.child_frame_id = 'cart_frame'
            cart_transform.transform.translation.x = map_point.point.x
            cart_transform.transform.translation.y = map_point.point.y
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

        t = TransformStamped()
        try:
            t = self.tf_buffer.lookup_transform('robot_base_link', 'cart_frame', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=3))
        except tf2_ros.TransformException:
            self.get_logger().warn("cart_frame not available in TF tree...")
            return

        self.get_logger().info("Moving to cart loading position...")

        # Rotate the robot towards the goal
        while rclpy.ok():
            t = self.tf_buffer.lookup_transform('robot_base_link', 'cart_frame', rclpy.time.Time())

            dx = t.transform.translation.x + 0.4
            dy = t.transform.translation.y

            yaw_error = math.atan2(dy, dx)
            if yaw_error > math.pi:
                yaw_error -= 2 * math.pi
            elif yaw_error < -math.pi:
                yaw_error += 2 * math.pi

            vel_msg = Twist()

            if abs(yaw_error) > 0.1:
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = 0.2 * yaw_error
                self.cmd_vel_pub.publish(vel_msg)
                # self.get_logger().info(f"Rotate-err {yaw_error:.2f}")
            else:
                # self.get_logger().info("Rotation completed, moving forward.")
                break

            rclpy.spin_once(self, timeout_sec=0.01)

        vel_msg = Twist()
        vel_msg.linear.x = 0.15
        vel_msg.angular.z = 0.0

        # Move forward X sec
        start_time = time.time()
        while rclpy.ok():
            end_time = time.time()
            elapsed_time = end_time - start_time
            if elapsed_time > 7.0:
                vel_msg.linear.x = 0.0
                self.cmd_vel_pub.publish(vel_msg)
                self.get_logger().info("Move completed.")
                return

            self.cmd_vel_pub.publish(vel_msg)
            rclpy.spin_once(self, timeout_sec=0.01)

    def move_back_up(self):
        """Move robot back to resume navigation."""

        start_x = self.robot_odom.pose.pose.position.x
        start_y = self.robot_odom.pose.pose.position.y

        q = self.robot_odom.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        while rclpy.ok():
            current_pose = self.robot_odom.pose.pose
            curr_x = current_pose.position.x
            curr_y = current_pose.position.y

            dx = curr_x - start_x
            dy = curr_y - start_y
            moved_distance = -(dx * math.cos(yaw) + dy * math.sin(yaw))
            error_distance = 1.5 - moved_distance

            vel_msg = Twist()

            if error_distance > 0.1:
                vel_msg.linear.x = -min(0.3, 1.0 * error_distance)
                vel_msg.angular.z = 0.0
                self.cmd_vel_pub.publish(vel_msg)
            else:
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = 0.0
                self.cmd_vel_pub.publish(vel_msg)
                return
            rclpy.spin_once(self, timeout_sec=0.01)

    def set_footprint(self, params_dict):
        """Set costmap robot footprint parameters dynamically."""

        for costmap in ['local_costmap/local_costmap', 'global_costmap/global_costmap']:
            cli = self.create_client(SetParameters, f'/{costmap}/set_parameters')

            if not cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn(f"Service {costmap}/set_parameters not available!")
                continue

            req = SetParameters.Request()
            req.parameters = []
            for name, value in params_dict.items():
                param = Parameter()
                param.name = name
                if isinstance(value, float):
                    param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value)
                elif isinstance(value, str):
                    param.value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=value)
                elif isinstance(value, list):
                    param.value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=str(value))
                req.parameters.append(param)

            future = cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

            if future.result() is not None:
                self.get_logger().info(f"Set parameters for {costmap}")
            else:
                self.get_logger().warn(f"Failed to set parameters for {costmap}")
 
    def perform_tasks(self):
        self.detach_pub.publish(String()) # Ensure elevator is not up
        for i, (x, y, yaw) in enumerate(self.task_route[1:], start=1):
            pose = self.create_pose(x, y, yaw)
            if i == 1:
                self.go_to_pose(pose, "Loading position")
                self.approach_cart()
                self.get_logger().info("Lifting the shelf")
                self.attach_pub.publish(String())
                time.sleep(2)
                self.attach_pub.publish(String()) # To ensure its up
                time.sleep(8)
                self.set_footprint(self.footprint_w_cart)
                self.get_logger().info("Shelf attached. Moving to Shipping position")
                self.move_back_up()
            else:
                self.go_to_pose(pose, "Shipping position")
                self.get_logger().info("At Shipping position. Detaching the shelf")
                self.detach_pub.publish(String())
                time.sleep(2)
                self.detach_pub.publish(String()) # To ensure its down
                time.sleep(8)
                self.set_footprint(self.footprint_normal)
                self.get_logger().info("Shelf detached. Returning to initial pose...")
                self.move_back_up()

    def return_to_start(self):
        self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.go_to_pose(self.initial_pose, "Initial position")

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

