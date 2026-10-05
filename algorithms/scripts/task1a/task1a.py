#!/usr/bin/env python3


'''
*****************************************************************************************
*
*        		===============================================
*           		        StrataCobot (SC) Theme (eYRC 2026-27)
*        		===============================================
*
*  This script should be used to implement Task 1A of StrataCobot (SC) Theme (eYRC 2026-27).
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
# Filename:		    task1a.py
# Functions:
#			        detect_ores, ore_tf.depthimagecb, ore_tf.colorimagecb, ore_tf.caminfocb,
#                   ore_tf.assign_ids, ore_tf.process_image, main
# Nodes:		    ore_tf_publisher
#			        Publishing Topics  - [ /tf ]
#                   Subscribing Topics - [ /camera/camera/color/image_raw,
#                                          /camera/camera/aligned_depth_to_color/image_raw,
#                                          /camera/camera/color/camera_info ]


################### IMPORT MODULES #######################

import os
import rclpy
from rclpy.executors import ExternalShutdownException
import sys
import cv2
import math
import tf2_ros
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped, PointStamped
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_point


##################### TASK CONSTANTS #######################

# Two ores of each type are spawned - six in all - told apart by an id of 1 or 2.
ore_types = ['azurite_ore', 'malachite_ore', 'vanadinite_ore']

# The RealSense topics. The depth image is ALIGNED to the colour image.
color_topic = '/camera/camera/color/image_raw'
depth_topic = '/camera/camera/aligned_depth_to_color/image_raw'
camera_info_topic = '/camera/camera/color/camera_info'

# The parent frame every ore transform is published against.
base_frame = 'base_link'

# HSV bounds per ore type (OpenCV: H 0-179, S/V 0-255). Measured on a saved sim frame:
# azurite H105 S230, malachite H74 S223, vanadinite orange H10 S252. The Mars terrain is
# also orange (H~8) but only S~160, so vanadinite is told apart by saturation alone.
# The ore_package is purple (H~150) and falls between them.
# TUNE: check against a saved frame (see SAVE_FRAME below) if a type is missed or leaks.
ore_hsv_ranges = {
    'azurite_ore':    [((95, 120, 25), (130, 255, 255))],
    'malachite_ore':  [((40, 120, 25), (85, 255, 255))],
    'vanadinite_ore': [((4, 220, 60), (16, 255, 255))],
}

min_ore_area_px = 80           # contours smaller than this are noise, not ores
morph_kernel = np.ones((5, 5), np.uint8)

# Ore collision box from models/<ore>/model.sdf: 0.1016 x 0.1016 x 0.0762 m.
ore_half_z = 0.0762 / 2.0

depth_window = 4               # median over a (2*4+1)^2 patch around the centre pixel

SHOW = bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))   # imshow aborts without a display
SAVE_FRAME = os.environ.get('SAVE_FRAME')   # set to a path to dump one colour frame for HSV tuning


##################### FUNCTION DEFINITIONS #######################

def detect_ores(image):
    '''
    Description:    Function to detect the ores present in a colour image frame and
                    return the pixel location and the type of each one found.

    Args:
        image                   (Image):    Input colour image frame received from the camera topic

    Returns:
        center_ore_list         (list):     Center pixel (cX, cY) of every ore detected in the frame
        ore_type_list           (list):     Type of each ore detected, taken from 'ore_types'
    '''

    center_ore_list = []
    ore_type_list = []

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    for ore_type in ore_types:
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for lower, upper in ore_hsv_ranges[ore_type]:
            mask |= cv2.inRange(hsv, np.array(lower), np.array(upper))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, morph_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, morph_kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [c for c in contours if cv2.contourArea(c) >= min_ore_area_px]
        # At most two ores per type exist; anything beyond the two largest blobs is clutter.
        for c in sorted(contours, key=cv2.contourArea, reverse=True)[:2]:
            M = cv2.moments(c)
            if M['m00'] == 0:
                continue
            center_ore_list.append((int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])))
            ore_type_list.append(ore_type)

    return center_ore_list, ore_type_list


##################### CLASS DEFINITION #######################

class ore_tf(Node):
    '''
    ___CLASS___

    Description:    Class which serves the purpose to detect the ores in the cell and
                    broadcast a transform for each one.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class ore_tf
        '''

        super().__init__(                                                               # registering node
            'ore_tf_publisher',
            parameter_overrides=[rclpy.parameter.Parameter(
                'use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        ############ Topic SUBSCRIPTIONS ############

        self.color_cam_sub = self.create_subscription(Image, color_topic, self.colorimagecb, 10)
        self.depth_cam_sub = self.create_subscription(Image, depth_topic, self.depthimagecb, 10)
        self.cam_info_sub = self.create_subscription(CameraInfo, camera_info_topic, self.caminfocb, 10)

        ############ Constructor VARIABLES/OBJECTS ############

        image_processing_rate = 0.2                                                     # rate of time to process image (seconds)
        self.bridge = CvBridge()                                                        # initialise CvBridge object for image conversion
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)                                    # object as transform broadcaster to send transform wrt some frame_id
        self.timer = self.create_timer(image_processing_rate, self.process_image)       # creating a timer based function which gets called on every 0.2 seconds (as defined by 'image_processing_rate' variable)

        self.cv_image = None                                                            # colour raw image variable (from colorimagecb())
        self.depth_image = None                                                         # depth image variable (from depthimagecb())
        self.cam_info = None                                                            # camera intrinsics variable (from caminfocb())

        self.optical_frame = None          # frame the colour image (and so the deprojection) lives in
        self.id_pixels = {t: {} for t in ore_types}   # ore_type -> {id: (u, v)} last pixel per id
        self.ore_positions = {}            # 'azurite_ore_1' -> (x, y, z) in base_frame, re-broadcast every cycle


    def depthimagecb(self, data):
        '''
        Description:    Callback function for the aligned depth camera topic.
                        Use this function to receive the depth image and convert it to a CV2 image.

        Args:
            data (Image):    Input depth image frame received from the aligned depth camera topic

        Returns:
        '''

        try:
            depth = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().warn(f'depth conversion failed: {e}')
            return
        depth = np.asarray(depth, dtype=np.float32)
        if data.encoding == '16UC1':        # millimetres; the sim publishes 32FC1 metres
            depth = depth / 1000.0
        self.depth_image = depth


    def colorimagecb(self, data):
        '''
        Description:    Callback function for the colour camera raw topic.
                        Use this function to receive the raw image and convert it to a CV2 image.

        Args:
            data (Image):    Input coloured raw image frame received from the image_raw camera topic

        Returns:
        '''

        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            self.optical_frame = data.header.frame_id
        except CvBridgeError as e:
            self.get_logger().warn(f'colour conversion failed: {e}')


    def caminfocb(self, data):
        '''
        Description:    Callback function for the camera info topic.
                        Use this function to receive the camera's intrinsic parameters.

        Args:
            data (CameraInfo):    Camera calibration published by the camera

        Returns:
        '''

        k = data.k
        self.cam_info = {'fx': k[0], 'fy': k[4], 'cx': k[2], 'cy': k[5]}


    def assign_ids(self, ore_type, detections):
        '''
        Description:    Give each detection of one ore type an id of 1 or 2 that stays the
                        same for the whole run.

        Args:
            ore_type    (str):      One of 'ore_types'
            detections  (list):     [((u, v), (x, y, z)), ...] pixel and base_frame point

        Returns:
            (list):     [(id, (u, v), (x, y, z)), ...]
        '''

        known = self.id_pixels[ore_type]

        if len(known) < 2 and len(detections) == 2:
            # First clean sighting of both: id 1 is the one nearer the arm base, matching the
            # spawn table (ore_1 rows sit closer to the arm than ore_2 rows).
            ordered = sorted(detections, key=lambda d: math.hypot(d[1][0], d[1][1]))
            known.update({1: ordered[0][0], 2: ordered[1][0]})
            return [(i + 1, d[0], d[1]) for i, d in enumerate(ordered)]

        if not known:
            # Only one seen so far: name it 1 until the second shows up.
            known[1] = detections[0][0]
            return [(1, detections[0][0], detections[0][1])]

        # Match each detection to the nearest already-named pixel; ores do not move.
        out, free = [], dict(known)
        for uv, p in detections:
            if not free:
                break
            i = min(free, key=lambda k: math.hypot(free[k][0] - uv[0], free[k][1] - uv[1]))
            known[i] = uv
            del free[i]
            out.append((i, uv, p))
        return out


    def process_image(self):
        '''
        Description:    Timer function used to detect the ores and publish a transform for
                        each one on its estimated position.

        Args:
        Returns:
        '''

        if self.cv_image is None or self.depth_image is None or self.cam_info is None:
            return

        image = self.cv_image.copy()
        depth = self.depth_image
        if SAVE_FRAME:
            cv2.imwrite(SAVE_FRAME, image)

        try:
            tf = self.tf_buffer.lookup_transform(base_frame, self.optical_frame, rclpy.time.Time())
        except Exception as e:                       # tree not filled in yet
            self.get_logger().warn(f'waiting for TF {base_frame} <- {self.optical_frame}: {e}',
                                   throttle_duration_sec=2.0)
            return

        centers, types = detect_ores(image)
        fx, fy, cx, cy = (self.cam_info[k] for k in ('fx', 'fy', 'cx', 'cy'))
        h, w = depth.shape[:2]

        per_type = {t: [] for t in ore_types}
        for (u, v), ore_type in zip(centers, types):
            patch = depth[max(v - depth_window, 0):min(v + depth_window + 1, h),
                          max(u - depth_window, 0):min(u + depth_window + 1, w)]
            patch = patch[np.isfinite(patch) & (patch > 0.0)]
            if patch.size == 0:
                continue
            z = float(np.median(patch))

            pt = PointStamped()
            pt.header.frame_id = self.optical_frame
            pt.point.x = (u - cx) * z / fx
            pt.point.y = (v - cy) * z / fy
            pt.point.z = z
            p = do_transform_point(pt, tf).point

            # The blob centre's depth hit lands on the ore's top face (the mast camera looks
            # down steeply), so the centre is half the box height straight below it.
            # Checked against Gazebo ground truth: stepping along the view ray instead
            # overshot by ~3.5 cm horizontally.
            centre = (p.x, p.y, p.z - ore_half_z)
            per_type[ore_type].append(((u, v), tuple(float(c) for c in centre)))

        for ore_type, detections in per_type.items():
            if not detections:
                continue
            for ore_id, (u, v), pos in self.assign_ids(ore_type, detections):
                name = f'{ore_type}_{ore_id}'
                self.ore_positions[name] = pos
                cv2.circle(image, (u, v), 5, (255, 255, 255), -1)
                cv2.putText(image, name, (u + 8, v - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                            (255, 255, 255), 1)

        # Broadcast every ore ever seen on every cycle, from its latest estimate.
        now = self.get_clock().now().to_msg()
        for name, (x, y, z) in self.ore_positions.items():
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = base_frame
            t.child_frame_id = name
            t.transform.translation.x = float(x)
            t.transform.translation.y = float(y)
            t.transform.translation.z = float(z)
            t.transform.rotation.w = 1.0
            self.br.sendTransform(t)

        self.get_logger().info(
            ' | '.join(f'{n}: ({x:.3f}, {y:.3f}, {z:.3f})'
                       for n, (x, y, z) in sorted(self.ore_positions.items())),
            throttle_duration_sec=2.0)

        if SHOW:
            cv2.imshow('ores', image)
            cv2.waitKey(1)


##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spins around for the ore_tf
                    class to perform its task
    '''

    rclpy.init(args=sys.argv)                                       # initialisation

    node = rclpy.create_node('ore_tf_process')                      # creating ROS node

    node.get_logger().info('Node created: Ore tf process')          # logging information

    ore_tf_class = ore_tf()                                         # creating a new object for class 'ore_tf'

    try:
        rclpy.spin(ore_tf_class)                                    # spining on the object to make it alive in ROS 2 DDS
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    ore_tf_class.destroy_node()                                     # destroy node after spin ends

    cv2.destroyAllWindows()

    rclpy.try_shutdown()                                            # shutdown process


if __name__ == '__main__':

    main()
