#!/usr/bin/env python3
'''Offline check of Task 1A detect_ores on a real Gazebo frame from the mast camera
(tools/frame_task1a.png, saved with SAVE_FRAME). Two of each ore must be found on the
conveyor, and nothing on the orange Mars terrain. Re-save the frame if the scene changes.
    python3 tools/sim_task1a.py'''
import os, sys, types, importlib.util
import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
for m in ['rclpy', 'rclpy.node', 'rclpy.executors', 'tf2_ros', 'cv_bridge', 'geometry_msgs', 'geometry_msgs.msg',
          'sensor_msgs', 'sensor_msgs.msg', 'tf2_geometry_msgs']:
    sys.modules.setdefault(m, types.ModuleType(m))
sys.modules['rclpy.node'].Node = object
sys.modules['rclpy.executors'].ExternalShutdownException = Exception
for m, n in [('cv_bridge', 'CvBridge'), ('cv_bridge', 'CvBridgeError'), ('geometry_msgs.msg', 'TransformStamped'),
             ('geometry_msgs.msg', 'PointStamped'), ('sensor_msgs.msg', 'CameraInfo'),
             ('sensor_msgs.msg', 'Image'), ('tf2_geometry_msgs', 'do_transform_point')]:
    setattr(sys.modules[m], n, object)
spec = importlib.util.spec_from_file_location('t', os.path.join(HERE, '..', 'algorithms', 'scripts', 'task1a', 'task1a.py'))
t = importlib.util.module_from_spec(spec); spec.loader.exec_module(t)


if __name__ == '__main__':
    img = cv2.imread(os.path.join(HERE, 'frame_task1a.png'))
    centers, types_ = t.detect_ores(img)
    for name in t.ore_types:
        got = [c for c, ty in zip(centers, types_) if ty == name]
        assert len(got) == 2, (name, got)
        assert all(440 < u < 600 and 210 < v < 380 for u, v in got), (name, got)   # ores sit on the conveyor
    print('ok  real frame: two of each ore found on the conveyor, terrain ignored')
