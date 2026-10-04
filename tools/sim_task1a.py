#!/usr/bin/env python3
'''Offline check of Task 1A detect_ores on a synthetic frame painted with the ore colours
from the .dae models (plus the purple ore_package, which must NOT be detected). Only checks
the colour plumbing; tune on a real frame (SAVE_FRAME=/tmp/f.png) for lighting.
    python3 tools/sim_task1a.py'''
import os, sys, types, importlib.util
import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
for m in ['rclpy', 'rclpy.node', 'tf2_ros', 'cv_bridge', 'geometry_msgs', 'geometry_msgs.msg',
          'sensor_msgs', 'sensor_msgs.msg', 'tf2_geometry_msgs']:
    sys.modules.setdefault(m, types.ModuleType(m))
sys.modules['rclpy.node'].Node = object
for m, n in [('cv_bridge', 'CvBridge'), ('cv_bridge', 'CvBridgeError'), ('geometry_msgs.msg', 'TransformStamped'),
             ('geometry_msgs.msg', 'PointStamped'), ('sensor_msgs.msg', 'CameraInfo'),
             ('sensor_msgs.msg', 'Image'), ('tf2_geometry_msgs', 'do_transform_point')]:
    setattr(sys.modules[m], n, object)
spec = importlib.util.spec_from_file_location('t', os.path.join(HERE, '..', 'algorithms', 'scripts', 'task1a', 'task1a.py'))
t = importlib.util.module_from_spec(spec); spec.loader.exec_module(t)

LINEAR_RGB = {   # diffuse colours from models/*/meshes/*.dae
    'azurite_ore': (0.001517, 0.03955, 0.1683),
    'malachite_ore': (0.003346, 0.7011, 0.08228),
    'vanadinite_ore': (0.5776, 0.02519, 0.0003),
    'ore_package': (0.2159, 0.0, 0.2159),
}


def bgr(rgb, shade):
    srgb = [1.055 * (c * shade) ** (1 / 2.4) - 0.055 if c * shade > 0.0031308 else 12.92 * c * shade for c in rgb]
    return tuple(int(255 * c) for c in reversed(srgb))


if __name__ == '__main__':
    for shade in (1.0, 0.5, 0.25):                  # bright, half-lit, shadowed
        img = np.full((480, 640, 3), (90, 110, 130), np.uint8)   # brownish ground
        expected = {}
        for row, name in enumerate(LINEAR_RGB):
            for col in range(2):
                u, v = 100 + col * 300, 60 + row * 110
                cv2.rectangle(img, (u - 25, v - 20), (u + 25, v + 20), bgr(LINEAR_RGB[name], shade), -1)
                expected.setdefault(name, []).append((u, v))
        centers, types_ = t.detect_ores(img)
        got = {}
        for c, ty in zip(centers, types_):
            got.setdefault(ty, []).append(c)
        assert 'ore_package' not in got
        for name in t.ore_types:
            assert sorted(got.get(name, [])) == sorted(expected[name]), (shade, name, got.get(name))
        print(f'ok  shade {shade}: all six ores found, package ignored')
