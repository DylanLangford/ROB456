#!/usr/bin/env python3

# Bill Smart, smartw@oregonstate.edu
#
# driver.py
# Drive the robot towards a goal, going around an object

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped
from math import atan2, tanh, sqrt, pi, fabs, cos, sin
import numpy as np

from std_msgs.msg import Header
from geometry_msgs.msg import TwistStamped, PointStamped
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from nav_targets.action import NavTarget

from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_geometry_msgs import do_transform_point

from rclpy.executors import MultiThreadedExecutor


class Lab3Driver(Node):

    def __init__(self, threshold=0.2):
        """We have parameters this time
        @param threshold - how close do you have to be before saying you're at the goal?
        """
        super().__init__('driver')

        self.goal = None
        self.threshold = threshold
        self.target_marker = None

        self.cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 1)
        self.target_pub = self.create_publisher(Marker, 'current_target', 1)

        self.lastdist = float('inf') # last distance
        self.stucktic = 0 # stuck counter
        self.stuckacceptable = 10 #amount of acceptable ticks to not have moved

        self.sub = self.create_subscription(
            LaserScan, 'base_scan', self.scan_callback, 10
        )

        self.tf_buffer = Buffer()
        self.transform_listener = TransformListener(self.tf_buffer, self)

        self.action_server = ActionServer(
            node=self,
            action_type=NavTarget,
            action_name="nav_target",
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_accept_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.action_callback
        )

        self.target = PointStamped()
        self.target.point.x = 0.0
        self.target.point.y = 0.0

        self.marker_timer = self.create_timer(1.0, self._marker_callback)

        self.count_since_last_scan = 0
        self.print_twist_messages = False
        self.print_distance_messages = False

    def _marker_callback(self):
        """Publishes the target so it shows up in RViz"""
        if not self.goal:
            if self.target_marker:
                self.target_marker.action = Marker.DELETE
                self.target_pub.publish(self.target_marker)
                self.target_marker = None
                self.get_logger().info(
                    "Driver: Had an existing target marker; removing"
                )
            return

        if not self.target_marker:
            self.target_marker = Marker()
            self.target_marker.header.frame_id = self.goal.header.frame_id
            self.target_marker.id = 0
            self.get_logger().info("Driver: Creating Marker")

        self.target_marker.header.stamp = self.get_clock().now().to_msg()
        self.target_marker.type = Marker.SPHERE
        self.target_marker.action = Marker.ADD
        self.target_marker.pose.position = self.goal.point
        self.target_marker.scale.x = 0.3
        self.target_marker.scale.y = 0.3
        self.target_marker.scale.z = 0.3
        self.target_marker.color.r = 0.0
        self.target_marker.color.g = 1.0
        self.target_marker.color.b = 0.0
        self.target_marker.color.a = 1.0

        self.target_pub.publish(self.target_marker)
        self.marker_timer.cancel()

    def zero_twist(self):
        t = TwistStamped()
        t.header.frame_id = 'base_link'
        t.header.stamp = self.get_clock().now().to_msg()
        return t

    def goal_accept_callback(self, goal_request: ServerGoalHandle):
        self.get_logger().info("Received a goal request")
        self.marker_timer.reset()
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Received a cancel request")
        self.goal = None
        self.cmd_pub.publish(self.zero_twist())
        self.marker_timer.reset()
        return CancelResponse.ACCEPT

    def close_enough(self):
        distance = self.distance_to_target()
        return distance < self.threshold 

    def distance_to_target(self):
        if self.target is None:
            return float('inf')
        return np.sqrt(
            self.target.point.x ** 2 +
            self.target.point.y ** 2
        )

    def action_callback(self, goal_handle: ServerGoalHandle):

        self.goal = PointStamped()
        self.goal.header = goal_handle.request.goal.header
        self.goal.point = goal_handle.request.goal.point

        result = NavTarget.Result()
        result.success = False

        self.set_target()
        
        self.lastdist = float('inf') # last distance
        self.stucktic = 0

        rate = self.create_rate(0.5)

        

        while not self.close_enough():

            if not self.goal:
                return result
            
            currentdist = self.distance_to_target()
            stucktolerance = 0.02


            if fabs(currentdist - self.lastdist) < stucktolerance:
                self.stucktic = self.stucktic + 1
            else:
                self.stucktic = 0

            self.lastdist = currentdist

            if self.stucktic > self.stuckacceptable:
                self.get_logger().info("Rob stuck :(")
                self.goal = None
                self.cmd_pub.publish(self.zero_twist())

                goal_handle.abort()
                return result


            feedback = NavTarget.Feedback()
            feedback.distance.data = self.distance_to_target()
            goal_handle.publish_feedback(feedback)

            rate.sleep()

        self.marker_timer.reset()
        self.goal = None
        self.cmd_pub.publish(self.zero_twist())

        goal_handle.succeed()
        result.success = True
        return result
    
    def set_target(self):
        """ Convert the goal into an x,y position (target) in the ROBOT's coordinate space
        @return the new target as a Point """

        if self.goal:
            # Transforms for all coordinate frames in the robot are stored in a transform tree
            # odom is the coordinate frame of the "world", base_link is the base link of the robot
            # A transform stores a rotation/translation to go from one coordinate system to the other
            transform = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))

            # This applies the transform to the Stamped Point
            # Note: This does not work, for reasons that are unclear to me
            self.target = do_transform_point(self.goal, transform)
            
            # This does the transform manually, by calculating the theta rotation from the quaternion
            euler_ang = -atan2(2 * transform.transform.rotation.z * transform.transform.rotation.w,
                               1.0 - 2 * transform.transform.rotation.z * transform.transform.rotation.z)
            
            # Translate to the base link's origin
            x = self.goal.point.x - transform.transform.translation.x
            y = self.goal.point.y - transform.transform.translation.y

            # Do the rotation
            rot_x = x * cos(euler_ang) - y * sin(euler_ang)
            rot_y = x * sin(euler_ang) + y * cos(euler_ang)

            self.target.point.x = rot_x
            self.target.point.y = rot_y
            if self.print_distance_messages:
                self.get_logger().info(f'Target relative to robot: ({self.target.point.x:.2f}, {self.target.point.y:.2f}), orig ({self.goal.point.x, self.goal.point.y})')
            
        else:
            if self.print_distance_messages:
                self.get_logger().info(f'No target to get distance to')
            self.target = None        
        
        # GUIDE: Calculate any additional variables here
        # Remember that the target's location is in its own coordinate frame at 0,0, angle 0 (x-axis)
        # YOUR CODE HERE

        return self.target

    def scan_callback(self, scan):

        self.count_since_last_scan = 0

        if self.goal:
            self.set_target()
            t = self.get_twist(scan)
        else:
            t = self.zero_twist()

        self.cmd_pub.publish(t)

    def get_obstacle(self, scan):

        num_readings = len(scan.ranges)
        scans = np.array(scan.ranges)
        thetas = np.linspace(scan.angle_min, scan.angle_max, num_readings)

        in_front_dists = []
        left_dists = []
        right_dists = []

        obstacle_threshold = .3

        for i in range(num_readings):
            y = scans[i] * np.sin(thetas[i])

            if -0.19 <= y <= 0.19:
                in_front_dists.append(scans[i])
            elif y < -0.19:
                left_dists.append(scans[i])
            elif y > 0.19:
                right_dists.append(scans[i])

        if not in_front_dists:
            return False, 0.0, 0.0

        front_range = np.min(in_front_dists)
        left_range = np.min(left_dists) if left_dists else float('inf')
        right_range = np.min(right_dists) if right_dists else float('inf')

        if front_range < obstacle_threshold:
            if left_range > right_range:
                return True, 0.0, np.pi / 2
            else:
                return True, 0.0, -np.pi / 2

        if left_range < obstacle_threshold or right_range < obstacle_threshold:
            if left_range < right_range:
                return True, 0.0, np.pi / 2
            else:
                return True, 0.0, -np.pi / 2

        return False, 0.0, 0.0

    def get_twist(self, scan):

        t = self.zero_twist()

        goal_angle = atan2(self.target.point.y, self.target.point.x)
        distance = self.distance_to_target()

        if distance > 1:
            t.twist.linear.x = 0.7
        else:
            t.twist.linear.x = 0.2 * np.tanh(distance)

        t.twist.angular.z = goal_angle

        obstacle_detected, object_x, object_z = self.get_obstacle(scan)

        if obstacle_detected:
            t.twist.linear.x = object_x
            t.twist.angular.z = object_z

        return t


def main(args=None):

    rclpy.init(args=args)

    driver = Lab3Driver()

    executor = MultiThreadedExecutor()
    executor.add_node(driver)
    executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
