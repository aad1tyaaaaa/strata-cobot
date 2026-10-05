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
          'sensor_msgs', 'sensor_msgs.msg']:
    sys.modules.setdefault(m, types.ModuleType(m))
sys.modules['rclpy.node'].Node = object
sys.modules['rclpy.executors'].ExternalShutdownException = Exception
for m, n in [('cv_bridge', 'CvBridge'), ('cv_bridge', 'CvBridgeError'), ('geometry_msgs.msg', 'TransformStamped'),
             ('sensor_msgs.msg', 'CameraInfo'),
             ('sensor_msgs.msg', 'Image')]:
    setattr(sys.modules[m], n, object)
spec = importlib.util.spec_from_file_location('t', os.path.join(HERE, '..', 'algorithms', 'scripts', 'task1a', 'task1a.py'))
t = importlib.util.module_from_spec(spec); spec.loader.exec_module(t)


if __name__ == '__main__':
    img = cv2.imread(os.path.join(HERE, 'frame_task1a.png'))
    centers, types_, _ = t.detect_ores(img)
    for name in t.ore_types:
        got = [c for c, ty in zip(centers, types_) if ty == name]
        assert len(got) == 2, (name, got)
        assert all(440 < u < 600 and 210 < v < 380 for u, v in got), (name, got)   # ores sit on the conveyor
    s = 0.5 ** 0.5   # 90 deg about z sends x to y
    assert np.allclose(t.quat_to_matrix(0, 0, s, s) @ [1, 0, 0], [0, 1, 0])
    print('ok  real frame: two of each ore found on the conveyor, terrain ignored')
