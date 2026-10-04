#!/usr/bin/env python3


'''
*****************************************************************************************
*
*        		===============================================
*           		        StrataCobot (SC) Theme (eYRC 2026-27)
*        		===============================================
*
*  This script should be used to implement Task 1B of StrataCobot (SC) Theme (eYRC 2026-27).
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
# Filename:		    task1b.py
# Functions:
#			        compute_twist, arm_waypoints.tcpposecb, arm_waypoints.jointstatecb,
#                   arm_waypoints.armstatuscb, arm_waypoints.switch_controller,
#                   arm_waypoints.publish_twist, arm_waypoints.process_waypoints, main
# Nodes:		    arm_waypoints_node
#			        Publishing Topics  - [ /delta_twist_cmds ]
#                   Subscribing Topics - [ /tcp_pose_raw, /joint_states, /arm_status ]
#                   Service clients    - [ /ur_arm_controller/switch_controller ]


################### IMPORT MODULES #######################

import rclpy
import sys
import math
import numpy as np
from rclpy.node import Node
from control_msgs.msg import JointJog
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import PoseStamped, TwistStamped
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32


##################### TASK CONSTANTS #######################

# Tool positions in base_link, in metres, in the order they must be reached. The tool
# stops at each one and holds it for at least two seconds. Copy the signs as they are:
# base_link is the UR7e's own frame, not the Gazebo world's.
waypoints = [
    (-0.4085, -0.5379, 0.1967),   # 1
    (-0.8000, -0.0005, 0.3967),   # 2
    (-0.7430,  0.5280, 0.1967),   # 3
    (-0.4097,  0.5280, 0.1967),   # 4
    (-0.0763,  0.5280, 0.1967),   # 5
]

# The two command interfaces. Only ONE is active at a time; messages to the other are
# accepted and ignored.
servo_ns = '/ur_arm_controller'
twist_controller = 'delta_twist_controller'
joint_controller = 'delta_joint_controller'

# The only frame a twist may be stamped with; any other is refused, not converted. Note
# 'base' is base_link turned through 180 degrees, not another name for it.
base_frame = 'base_link'

# JointJog velocities are matched to these names, in this order.
joint_names = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]

# What the servo accepts. A command above one of these is dropped WHOLE, not clamped.
cap_linear_mps = 0.15     # magnitude of a twist's linear part
cap_angular_rps = 0.35    # magnitude of its angular part
cap_joint_rps = 0.35      # per joint

# Dead-man switch: the arm stops this long after the last message it received.
command_timeout_s = 0.15

# ---- Controller tuning ----
# Commands stay a margin under the caps above, since anything over them is dropped whole.
max_linear_mps = 0.12
max_angular_rps = 0.30
kp_linear = 1.5               # (m/s) per m of position error
kp_angular = 1.2              # (rad/s) per rad of tool-axis error
# Linear speed may only rise this fast. Waypoint 3 is reached with the elbow nearly
# straight; leaving it at full speed at once needs several rad/s at the elbow.
max_linear_accel = 0.05       # m/s^2

# Desired direction of the tool's z axis in base_link: pointing straight down, the pick
# pose. All five waypoints are reachable this way (checked with UR7e IK offline).
tool_axis_target = np.array([0.0, 0.0, -1.0])

# Translation waits while the tool axis is more than this far off (only bites at the
# start, where the arm begins with the tool horizontal): rotate first, then move.
orient_gate_rad = 0.35

pos_tolerance_m = 0.01        # inside this the waypoint counts as reached
ang_tolerance_rad = 0.05
hold_time_s = 2.5             # task asks for >= 2 s; margin for clock jitter


##################### FUNCTION DEFINITIONS #######################

def compute_twist(position, quat_xyzw, target, prev_linear, dt):
    '''
    Description:    Proportional Cartesian control law for one cycle.

    Args:
        position    (array):    Tool position in base_link (x, y, z)
        quat_xyzw   (array):    Tool orientation in base_link (x, y, z, w)
        target      (tuple):    Waypoint (x, y, z) in base_link
        prev_linear (array):    Linear velocity commanded last cycle
        dt          (float):    Control period, seconds

    Returns:
        linear      (array):    Linear velocity, already capped
        angular     (array):    Angular velocity, already capped
        dist        (float):    Distance to the target
        ang_err     (float):    Angle between the tool axis and 'tool_axis_target'
    '''

    error = np.asarray(target, dtype=float) - np.asarray(position, dtype=float)
    dist = float(np.linalg.norm(error))

    # Rotation vector that turns the tool's own z axis 'a' onto the target axis 'b'.
    a = Rotation.from_quat(quat_xyzw).as_matrix()[:, 2]
    b = tool_axis_target
    c = np.cross(a, b)
    s = float(np.linalg.norm(c))
    ang_err = math.atan2(s, float(np.dot(a, b)))
    if s > 1e-9:
        rot = c / s * ang_err
    elif ang_err > 1.0:           # exactly opposite: any axis perpendicular to 'a' works
        rot = Rotation.from_quat(quat_xyzw).as_matrix()[:, 0] * ang_err
    else:
        rot = np.zeros(3)

    angular = kp_angular * rot
    linear = kp_linear * error if ang_err < orient_gate_rad else np.zeros(3)

    # Cap by scaling the WHOLE vector, so the direction is kept.
    for v, cap in ((linear, max_linear_mps), (angular, max_angular_rps)):
        n = np.linalg.norm(v)
        if n > cap:
            v *= cap / n

    # Ramp speeding up only; slowing down stays immediate so the tool cannot overshoot.
    n = np.linalg.norm(linear)
    allowed = np.linalg.norm(prev_linear) + max_linear_accel * dt
    if n > allowed:
        linear = linear * (allowed / n)

    return linear, angular, dist, ang_err


##################### CLASS DEFINITION #######################

class arm_waypoints(Node):
    '''
    ___CLASS___

    Description:    Class which serves the purpose to drive the UR7e's tool through the
                    given waypoints using the arm's velocity command interfaces.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class arm_waypoints
        '''

        # use_sim_time is set here, not on the command line, so this node runs on the
        # simulation clock however it is started.
        super().__init__(                                                               # registering node
            'arm_waypoints_node',
            parameter_overrides=[rclpy.parameter.Parameter(
                'use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        ############ Topic PUBLISHERS ############

        self.twist_pub = self.create_publisher(TwistStamped, '/delta_twist_cmds', 10)    # end-effector velocity, in base_link
        self.joint_pub = self.create_publisher(JointJog, '/delta_joint_cmds', 10)        # per-joint velocity

        ############ Topic SUBSCRIPTIONS ############

        self.tcp_sub = self.create_subscription(PoseStamped, '/tcp_pose_raw', self.tcpposecb, 20)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.jointstatecb, 50)
        self.status_sub = self.create_subscription(Int32, '/arm_status', self.armstatuscb, 10)

        ############ Constructor VARIABLES/OBJECTS ############

        control_rate = 0.05                                                             # rate of time to run one control cycle (seconds)
        self.control_rate = control_rate
        self.switch_cli = self.create_client(                                           # client used to pick which command topic is live
            SwitchController, f'{servo_ns}/switch_controller')

        self.tcp_pose = None                                                            # tool pose variable (from tcpposecb())
        self.joint_angles = None                                                        # joint feedback variable (from jointstatecb())
        self.arm_status = None                                                          # arm state code variable (from armstatuscb())

        self.wp_index = 0              # waypoint currently being driven to
        self.hold_start = None         # sim time the tool entered tolerance of it, or None
        self.done = False
        self.last_linear = np.zeros(3)   # for the acceleration ramp

        # Only the twist interface is used, so switch once here, where blocking is safe.
        self.switch_controller('twist_controller')

        self.timer = self.create_timer(control_rate, self.process_waypoints)            # creating a timer based function which gets called on every 0.05 seconds (as defined by 'control_rate' variable)


    def tcpposecb(self, data):
        '''
        Description:    Callback function for the tool pose topic.
                        Use this function to receive where the tool currently is.

        Args:
            data (PoseStamped):    Pose of the tool, reported in base_link

        Returns:
        '''

        p, q = data.pose.position, data.pose.orientation
        self.tcp_pose = (np.array([p.x, p.y, p.z]), np.array([q.x, q.y, q.z, q.w]))


    def jointstatecb(self, data):
        '''
        Description:    Callback function for the joint states topic.
                        Use this function to receive the current angle of each joint.

        Args:
            data (JointState):    Joint feedback published by the arm

        Returns:
        '''

        # Matched BY NAME; a message missing any arm joint is skipped, not zero-filled.
        try:
            self.joint_angles = [data.position[data.name.index(j)] for j in joint_names]
        except (ValueError, IndexError):
            pass


    def armstatuscb(self, data):
        '''
        Description:    Callback function for the arm status topic.
                        Use this function to receive the arm's current state code.

        Args:
            data (Int32):    One state code describing what the arm is doing

        Returns:
        '''

        if data.data != self.arm_status:
            log = self.get_logger().info if data.data == 0 else self.get_logger().warn
            log(f'/arm_status -> {data.data} (see /arm_status_detail)')
        self.arm_status = data.data


    def switch_controller(self, controller):
        '''
        Description:    Function to make one of the arm's two command interfaces the active
                        one, so that commands published to it are acted on.

        Args:
            controller  (str):      Name of the controller to activate, either
                                    'twist_controller' or 'joint_controller'

        Returns:
            success     (bool):     Whether the controller was activated
        '''

        on, off = ((twist_controller, joint_controller) if controller == 'twist_controller'
                   else (joint_controller, twist_controller))

        if not self.switch_cli.wait_for_service(timeout_sec=20.0):
            self.get_logger().error(f'{servo_ns}/switch_controller not available')
            return False

        # The first attempt is expected to fail while the servo comes up; STRICT then
        # BEST_EFFORT, since STRICT refuses if 'on' is already the active one.
        for attempt in range(6):
            req = SwitchController.Request()
            req.activate_controllers = [on]
            req.deactivate_controllers = [off]
            req.strictness = (SwitchController.Request.STRICT if attempt % 2 == 0
                              else SwitchController.Request.BEST_EFFORT)
            future = self.switch_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            if future.done() and future.result() is not None and future.result().ok:
                self.get_logger().info(f'{on} active (attempt {attempt + 1})')
                return True
            self.get_logger().warn(f'switch to {on} failed (attempt {attempt + 1}), retrying')

        self.get_logger().error(f'could not activate {on}; commands may be ignored')
        return False


    def publish_twist(self, linear, angular):
        '''
        Description:    Publish one TwistStamped in base_frame.

        Args:
            linear      (array):    m/s
            angular     (array):    rad/s

        Returns:
        '''

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = base_frame
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = (float(v) for v in linear)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = (float(v) for v in angular)
        self.twist_pub.publish(msg)


    def process_waypoints(self):
        '''
        Description:    Timer function used to drive the tool through the waypoints.

        Args:
        Returns:
        '''

        zero = np.zeros(3)

        # Publish on every tick, zero included, so the dead-man switch never trips mid-run.
        if self.done or self.tcp_pose is None or self.joint_angles is None or self.arm_status is None:
            self.publish_twist(zero, zero)
            return

        target = waypoints[self.wp_index]
        linear, angular, dist, ang_err = compute_twist(*self.tcp_pose, target,
                                                       self.last_linear, self.control_rate)
        self.last_linear = linear
        self.publish_twist(linear, angular)

        # Keep correcting during the hold: the servo ramps down and settles past the pose, so
        # the clock only runs while the tool is actually inside tolerance.
        now = self.get_clock().now().nanoseconds * 1e-9
        if dist < pos_tolerance_m and ang_err < ang_tolerance_rad:
            if self.hold_start is None:
                self.hold_start = now
                self.get_logger().info(f'waypoint {self.wp_index + 1} reached, holding')
            elif now - self.hold_start >= hold_time_s:
                self.get_logger().info(f'waypoint {self.wp_index + 1} held {hold_time_s:.1f}s')
                self.hold_start = None
                self.wp_index += 1
                if self.wp_index == len(waypoints):
                    self.done = True
                    self.get_logger().info('all waypoints done')
                    self.publish_twist(zero, zero)
        else:
            self.hold_start = None

        self.get_logger().info(
            f'wp {self.wp_index + 1}/{len(waypoints)} dist {dist:.4f} m, axis err {math.degrees(ang_err):.1f} deg',
            throttle_duration_sec=1.0)


##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spins around for the
                    arm_waypoints class to perform its task
    '''

    rclpy.init(args=sys.argv)                                       # initialisation

    node = rclpy.create_node('arm_waypoints_process')               # creating ROS node

    node.get_logger().info('Node created: Arm waypoints process')   # logging information

    arm_waypoints_class = arm_waypoints()                           # creating a new object for class 'arm_waypoints'

    try:
        rclpy.spin(arm_waypoints_class)                             # spining on the object to make it alive in ROS 2 DDS
    except KeyboardInterrupt:
        pass

    arm_waypoints_class.destroy_node()                              # destroy node after spin ends

    rclpy.try_shutdown()                                            # shutdown process


if __name__ == '__main__':

    main()
