#!/usr/bin/python3

from pyngrok import ngrok
import urllib.request as ur
from pyngrok import conf, ngrok
from odrive.enums import *
import time
import numpy as np
from geometry_msgs.msg import Twist
import argparse
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped, Twist
import tf_transformations
import math
import rclpy
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import Pose, TransformStamped
from rclpy.node import Node

goal = [
    # [60.11050796508789, -25.341800689697266],
    # [19.081645965576172, -84.20426940917969],
    [-56.13334655761719, -33.43186569213867],
    [42.272151947021484, -6.5556159019470215],
    [60.11050796508789, -25.341800689697266],
    [19.081645965576172, -84.20426940917969],
    [-56.13334655761719, -33.43186569213867],
    [42.272151947021484, -6.5556159019470215],
]

goal_index = 0
goal_flag = 0

class Purepursuit(Node):
    def __init__(self):
        super().__init__('carver_bringup')
        self.action_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel_purepursuit', 10)
        self.goal_marker_publisher = self.create_publisher(PoseStamped, 'goal_marker', 10)

        self.lookahead_distance = 5.0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_timer(0.05, self.pure_pursuit_controller)
        self.path = None
        self.current_pose_index = 0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.steering_angle = 0.0
        self.active_time = time.time()

        self.old_nearest_point_index = None
        self.PreTP_index = 0
        self.reachgoal = False
    def search_target_index(self, path, robot_pose):
        if self.old_nearest_point_index is None:
            dx = []
            dy = []
            for i in range(len(self.path)):
                dx.append(robot_pose.position.x - self.path[i].pose.position.x)
                dy.append(robot_pose.position.y - self.path[i].pose.position.y)
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = self.distance_between_points(robot_pose.position, path[ind].pose.position)
            while True:
                try:
                    distance_next_index = self.distance_between_points(robot_pose.position, path[ind + 1].pose.position)
                except IndexError:
                    return ind + 1
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 # if (ind + 1) < len(self.path) else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind
        while self.lookahead_distance > self.distance_between_points(robot_pose.position, path[ind].pose.position):
            if (ind + 1) >= len(self.path):
                ind += 1
                break
            ind += 1

        return ind

    def pure_pursuit_controller(self):
        global goal_flag
        if self.path is not None and self.current_pose_index < len(self.path):
            robot_pose = self.get_robot_pose()
            if robot_pose is not None:
                TP_index = self.search_target_index(self.path, robot_pose)
                if TP_index is not None:
                    print("check TP_index", TP_index)
                    # if self.PreTP_index >= TP_index:
                    #     TP_index = self.PreTP_index
                    if TP_index < len(self.path):
                        TP = self.path[TP_index]
                        linear_vel, steer_angle = self.calculate_velocities(robot_pose, TP)
                        self.publish_velocity(linear_vel, steer_angle)
                        goal_flag = 0
                    else: # reached final goal
                        if goal_flag == 0:
                            self.reachgoal = True
                            TP = self.path[-1]
                            TP_index = len(self.path) - 1
                            self.publish_velocity(0.0, 0.0)
                            self.next_goal()
                            goal_flag = 1
                try:
                    print(self.reachgoal, TP_index , "/", len(self.path), linear_vel, steer_angle)
                except Exception as e:
                    print(e)
                    pass
                self.PreTP_index = TP_index
        else:
            self.publish_velocity(0.0, 0.0)
            pass

    def send_goal(self, pose):
        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = pose
        self.action_client.wait_for_server()
        self.future = self.action_client.send_goal_async(goal_msg)
        self.future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Path received')
        self.follow_path(result.path.poses)

    def follow_path(self, path):
        self.path = path

    def calculate_velocities(self, robot_pose, goal_point):
        max_linear_velocity = 9.0  
        # max_angular_velocity = 0.60 
        angle_to_goal = math.atan2(goal_point.pose.position.y - robot_pose.position.y,
                                   goal_point.pose.position.x - robot_pose.position.x)
        
        _, _, yaw = tf_transformations.euler_from_quaternion([robot_pose.orientation.x,
                                                              robot_pose.orientation.y,
                                                              robot_pose.orientation.z,
                                                              robot_pose.orientation.w])
        heading_error = self.normalize_angle(angle_to_goal - yaw)
        self.linear_velocity = max_linear_velocity
        # print(goal_point.pose.position.x, goal_point.pose.position.y, robot_pose.position.x, robot_pose.position.y)
        self.steering_angle = math.atan2(2.0 * 1.3 * math.sin(heading_error)/self.lookahead_distance,1.0)
        return self.linear_velocity, self.steering_angle
    
    def publish_velocity(self, linear_vel, angular_vel):
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.velocity_publisher.publish(twist)

    def is_goal_reached(self, robot_pose, goal_pose):
        return self.distance_between_points(robot_pose.position, goal_pose.pose.position) <= self.goal_threshold

    def distance_between_points(self, point1, point2):
        return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)

    def normalize_angle(self, angle):
        if angle > math.pi:
            angle -= 2.0 * math.pi
        if angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def get_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            pose = Pose()
            pose.position.x = trans.transform.translation.x
            pose.position.y = trans.transform.translation.y
            pose.position.z = trans.transform.translation.z
            pose.orientation = trans.transform.rotation
            return pose
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error('Could not transform from base_link to map: %s' % str(e))
            return None
        
    def start_task(self, goal_x, goal_y):
        self.current_pose_index = 0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.steering_angle = 0.0
        self.active_time = time.time()

        self.old_nearest_point_index = None
        self.PreTP_index = 0
        self.path = None
        self.reachgoal = False

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position.x = goal_x
        goal_pose.pose.position.y = goal_y
        self.send_goal(goal_pose)

    def next_goal(self):
        global goal, goal_index
        goal_index += 1
        if goal_index < len(goal):
            self.start_task(goal[goal_index][0], goal[goal_index][1])
        
   
def main(args=None):
    rclpy.init(args=args)
    node = Purepursuit()
    node.start_task(goal[0][0], goal[0][1])
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()