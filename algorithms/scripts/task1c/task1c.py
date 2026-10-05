#!/usr/bin/env python3


'''
*****************************************************************************************
*
*        		===============================================
*           		        StrataCobot (SC) Theme (eYRC 2026-27)
*        		===============================================
*
*  This script should be used to implement Task 1C of StrataCobot (SC) Theme (eYRC 2026-27).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:          [ Team-ID ]
# Author List:		[ Aaditya Jaiswar, Prathamesh Rajesh Aarya ]
# Filename:		    task1c.py
# Functions:
#			        wrap, choose_heading, compute_cmd, ebot_nav.pathcb, ebot_nav.odomcb,
#                   ebot_nav.scancb, ebot_nav.publish_cmd, ebot_nav.process_navigation, main
# Nodes:		    ebot_nav_node
#			        Publishing Topics  - [ /cmd_vel ]
#                   Subscribing Topics - [ /ebot_path, /odom, /scan, /tf, /tf_static ]


################### IMPORT MODULES #######################

import rclpy
from rclpy.executors import ExternalShutdownException
import sys
import math
import numpy as np
import tf2_ros
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.qos import (qos_profile_sensor_data, QoSProfile, HistoryPolicy,
                       ReliabilityPolicy, DurabilityPolicy)
from sensor_msgs.msg import LaserScan


##################### TASK CONSTANTS #######################

# The route is published here ONCE, before your node starts, and held for late
# subscribers. An ordinary subscription waits forever for a message already sent.
path_topic = '/ebot_path'

odom_topic = '/odom'
scan_topic = '/scan'
cmd_topic = '/cmd_vel'

# What /cmd_vel accepts. The base clips anything beyond these, and clipping one of the
# two changes their ratio - which is the arc the base actually drives.
max_linear_mps = 0.5
max_angular_rps = 1.0

# ---- eBot geometry (ebot_description.xacro) ----
# Collision box 0.62 x 0.49 m, centred on the base; the wheels reach out to about the
# same half-width, so 0.245 m is the widest part either side of the centre line.
robot_half_width = 0.245
robot_front = 0.31             # base centre to front face

# ---- Tuning ----
side_margin = 0.16             # corridor half-width 0.405 > 0.395, the corner radius it sweeps turning on the spot
lookahead_m = 1.2              # how far ahead a heading has to be clear to be taken
stop_gap_m = 0.15              # closer than this in front: no forward motion at all
heading_step_rad = math.radians(5)
kp_angular = 1.5               # (rad/s) per rad of heading error
k_slow_free = 0.8              # (m/s) per m of clear space ahead beyond stop_gap_m
k_slow_goal = 0.8              # (m/s) per m to the final waypoint
turn_in_place_rad = math.radians(60)   # heading error above this: rotate, do not drive
waypoint_tolerance_m = 0.25    # intermediate waypoints
final_tolerance_m = 0.15       # the last one


##################### FUNCTION DEFINITIONS #######################

def wrap(angle):
    '''
    Description:    Wrap an angle to [-pi, pi].
    '''
    return math.atan2(math.sin(angle), math.cos(angle))


def choose_heading(points, goal_heading, goal_dist, fov):
    '''
    Description:    Pick the heading, relative to the base, closest to 'goal_heading'
                    along which a corridor as wide as the eBot is clear far enough.

    Args:
        points          (array):    N x 2 obstacle points in the base frame (x ahead, y left)
        goal_heading    (float):    Heading to the waypoint, relative to the base, rad
        goal_dist       (float):    Distance to the waypoint, m
        fov             (float):    Half-width of the lidar's view, rad; nothing outside it
                                    is known, so nothing outside it is chosen

    Returns:
        heading         (float):    Chosen relative heading, rad
        free            (float):    Clear distance ahead of the front face along it, m
    '''

    candidates = np.arange(-fov, fov + 1e-9, heading_step_rad)
    half = robot_half_width + side_margin
    if len(points):
        c, s = np.cos(candidates)[:, None], np.sin(candidates)[:, None]
        along = c * points[:, 0] + s * points[:, 1]                 # K x N
        lateral = -s * points[:, 0] + c * points[:, 1]
        blocked = (along > 0.0) & (np.abs(lateral) < half)
        free = np.where(blocked, along, np.inf).min(axis=1) - robot_front
    else:
        free = np.full(len(candidates), np.inf)

    need = min(lookahead_m, max(goal_dist - robot_front, 0.0) + stop_gap_m)
    ok = free > need
    if ok.any():
        # Clear: the one nearest the goal direction.
        i = int(np.argmin(np.where(ok, np.abs(candidates - goal_heading), np.inf)))
    else:
        # Nothing fully clear: the most open way, ties broken towards the goal.
        i = int(np.argmax(free - 0.01 * np.abs(candidates - goal_heading)))
    return float(candidates[i]), float(free[i])


def compute_cmd(heading, free, goal_dist, is_final):
    '''
    Description:    Turn a chosen relative heading into (linear, angular), inside the
                    limits and with their ratio kept.

    Returns:
        linear, angular     (float, float)
    '''

    angular = kp_angular * heading
    if abs(heading) > turn_in_place_rad or free < stop_gap_m:
        linear = 0.0
    else:
        linear = max_linear_mps * math.cos(heading) ** 2           # slower the worse it points
        linear = min(linear, k_slow_free * (free - stop_gap_m))
        if is_final:
            linear = min(linear, k_slow_goal * goal_dist)
        linear = max(linear, 0.0)

    # Scale both together if either is over its limit, so the arc is the one asked for.
    scale = max(1.0, abs(linear) / max_linear_mps, abs(angular) / max_angular_rps)
    return linear / scale, angular / scale


##################### CLASS DEFINITION #######################

class ebot_nav(Node):
    '''
    ___CLASS___

    Description:    Class which serves the purpose to drive the eBot along the route
                    published on /ebot_path, through its waypoints, in order.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class ebot_nav
        '''

        # use_sim_time is set here, not on the command line, so this node runs on the
        # simulation clock however it is started.
        super().__init__(                                                               # registering node
            'ebot_nav_node',
            parameter_overrides=[rclpy.parameter.Parameter(
                'use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        ############ Topic SUBSCRIPTIONS ############

        # The route is latched: match it with RELIABLE + TRANSIENT_LOCAL so the message
        # published before this node started is still delivered.
        path_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_sub = self.create_subscription(Path, path_topic, self.pathcb, path_qos)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odomcb, 10)
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scancb, qos_profile_sensor_data)

        ############ Topic PUBLISHERS ############

        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)                      # the eBot drives on what you publish here

        ############ Constructor VARIABLES/OBJECTS ############

        control_rate = 0.05                                                             # rate of time to run one control cycle (seconds)
        self.timer = self.create_timer(control_rate, self.process_navigation)           # creating a timer based function which gets called on every 0.05 seconds (as defined by 'control_rate' variable)

        self.route = None                                                               # the route to drive (from pathcb())
        self.odom = None                                                                # where the base is (from odomcb())
        self.scan = None                                                                # what the lidar sees (from scancb())

        self.wp_index = 0              # waypoint currently being driven to
        self.done = False
        self.base_frame = None         # odom's child frame, the frame the base is steered in
        self.lidar_pose = None         # (x, y, yaw) of the lidar in base_frame, from TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)


    def pathcb(self, data):
        '''
        Description:    Callback function for the route topic.
                        Use this function to receive the waypoints the eBot has to drive.

        Args:
            data (Path):    The route, as a sequence of poses

        Returns:
        '''

        if self.route is not None or not data.poses:    # take the route once
            return
        self.route = [(p.pose.position.x, p.pose.position.y) for p in data.poses]
        self.get_logger().info(
            f'route: {len(self.route)} waypoints in "{data.header.frame_id}": '
            + ', '.join(f'({x:.2f}, {y:.2f})' for x, y in self.route))


    def odomcb(self, data):
        '''
        Description:    Callback function for the odometry topic.
                        Use this function to receive where the base currently is.

        Args:
            data (Odometry):    Pose and velocity of the base

        Returns:
        '''

        p, q = data.pose.pose.position, data.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.odom = (p.x, p.y, yaw)
        self.base_frame = data.child_frame_id


    def scancb(self, data):
        '''
        Description:    Callback function for the lidar topic.
                        Use this function to receive what the lidar currently sees.

        Args:
            data (LaserScan):    One lidar sweep

        Returns:
        '''

        if self.lidar_pose is None:
            if self.base_frame is None:
                return
            try:
                tf = self.tf_buffer.lookup_transform(self.base_frame, data.header.frame_id,
                                                     rclpy.time.Time())
            except Exception as e:
                self.get_logger().warn(f'waiting for TF {self.base_frame} <- {data.header.frame_id}: {e}',
                                       throttle_duration_sec=2.0)
                return
            t, q = tf.transform.translation, tf.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            self.lidar_pose = (t.x, t.y, yaw)
            self.get_logger().info(f'lidar at ({t.x:.3f}, {t.y:.3f}) yaw {yaw:.3f} in {self.base_frame}')

        ranges = np.asarray(data.ranges, dtype=float)
        angles = data.angle_min + np.arange(len(ranges)) * data.angle_increment
        valid = np.isfinite(ranges) & (ranges >= data.range_min) & (ranges <= data.range_max)
        r, a = ranges[valid], angles[valid]

        # Beams are measured from the lidar; move them to the base centre.
        lx, ly, lyaw = self.lidar_pose
        points = np.column_stack((lx + r * np.cos(a + lyaw), ly + r * np.sin(a + lyaw)))
        fov = min(abs(data.angle_min), abs(data.angle_max))
        self.scan = (points, fov)


    def publish_cmd(self, linear, angular):
        '''
        Description:    Publish one Twist on /cmd_vel.
        '''

        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)


    def process_navigation(self):
        '''
        Description:    Timer function used to drive the eBot along the route.

        Args:
        Returns:
        '''

        # Nothing here may raise: a dead node leaves the base coasting on its last command.
        try:
            if self.done or self.route is None or self.odom is None or self.scan is None:
                self.publish_cmd(0.0, 0.0)
                return

            x, y, yaw = self.odom
            n = len(self.route)

            # Advance past every waypoint already reached, in order.
            while self.wp_index < n:
                tx, ty = self.route[self.wp_index]
                is_final = self.wp_index == n - 1
                tol = final_tolerance_m if is_final else waypoint_tolerance_m
                if math.hypot(tx - x, ty - y) >= tol:
                    break
                self.get_logger().info(f'waypoint {self.wp_index + 1}/{n} reached at ({x:.2f}, {y:.2f})')
                self.wp_index += 1

            if self.wp_index == n:
                self.done = True
                self.publish_cmd(0.0, 0.0)
                self.get_logger().info('route complete, base stopped')
                return

            tx, ty = self.route[self.wp_index]
            dist = math.hypot(tx - x, ty - y)
            goal_heading = wrap(math.atan2(ty - y, tx - x) - yaw)
            points, fov = self.scan

            if abs(goal_heading) > fov:
                # Behind the lidar's view: turn towards it on the spot first.
                heading, free = goal_heading, 0.0
            else:
                heading, free = choose_heading(points, goal_heading, dist, fov)

            linear, angular = compute_cmd(heading, free, dist, self.wp_index == n - 1)
            self.publish_cmd(linear, angular)

            self.get_logger().info(
                f'wp {self.wp_index + 1}/{n} dist {dist:.2f} goal {math.degrees(goal_heading):.0f} deg '
                f'chosen {math.degrees(heading):.0f} deg free {free:.2f} m cmd ({linear:.2f}, {angular:.2f})',
                throttle_duration_sec=1.0)

        except Exception as e:
            self.publish_cmd(0.0, 0.0)
            self.get_logger().error(f'navigation cycle failed: {e!r}', throttle_duration_sec=1.0)


##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spins around for the
                    ebot_nav class to perform its task
    '''

    rclpy.init(args=sys.argv)                                       # initialisation

    node = rclpy.create_node('ebot_nav_process')                    # creating ROS node

    node.get_logger().info('Node created: eBot navigation process') # logging information

    ebot_nav_class = ebot_nav()                                     # creating a new object for class 'ebot_nav'

    try:
        rclpy.spin(ebot_nav_class)                                  # spining on the object to make it alive in ROS 2 DDS
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            ebot_nav_class.publish_cmd(0.0, 0.0)                    # never leave the base coasting
        except Exception:
            pass

    ebot_nav_class.destroy_node()                                   # destroy node after spin ends

    rclpy.try_shutdown()                                            # shutdown process


if __name__ == '__main__':

    main()
