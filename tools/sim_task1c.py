#!/usr/bin/env python3
'''Offline check of the Task 1C controller: a unicycle eBot, synthetic +/-90 deg lidar and
round rocks dropped on the route. No ROS needed. Fails (assert) if it collides or does not
finish the route in order.   python3 tools/sim_task1c.py'''
import math, os, sys, types, importlib.util
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
for m in ['rclpy', 'rclpy.node', 'rclpy.executors', 'rclpy.qos', 'tf2_ros', 'geometry_msgs', 'geometry_msgs.msg',
          'nav_msgs', 'nav_msgs.msg', 'sensor_msgs', 'sensor_msgs.msg']:
    sys.modules.setdefault(m, types.ModuleType(m))
sys.modules['rclpy.node'].Node = object
sys.modules['rclpy.executors'].ExternalShutdownException = Exception
for n in ['qos_profile_sensor_data', 'QoSProfile', 'HistoryPolicy', 'ReliabilityPolicy', 'DurabilityPolicy']:
    setattr(sys.modules['rclpy.qos'], n, None)
for m, n in [('geometry_msgs.msg', 'Twist'), ('nav_msgs.msg', 'Odometry'), ('nav_msgs.msg', 'Path'),
             ('sensor_msgs.msg', 'LaserScan')]:
    setattr(sys.modules[m], n, object)
spec = importlib.util.spec_from_file_location('t', os.path.join(HERE, '..', 'algorithms', 'scripts', 'task1c', 'task1c.py'))
t = importlib.util.module_from_spec(spec); spec.loader.exec_module(t)

LIDAR_X = 0.2555                      # rplidar_joint, ebot_description.xacro
BEAMS = np.linspace(-1.57, 1.57, 360)


def scan(x, y, yaw, rocks):
    ox, oy = x + LIDAR_X * math.cos(yaw), y + LIDAR_X * math.sin(yaw)
    r = np.full(len(BEAMS), np.inf)
    for cx, cy, rad in rocks:          # ray / circle intersection
        dx, dy = np.cos(BEAMS + yaw), np.sin(BEAMS + yaw)
        fx, fy = ox - cx, oy - cy
        b = fx * dx + fy * dy
        disc = b * b - (fx * fx + fy * fy - rad * rad)
        hit = disc >= 0
        d = -b - np.sqrt(np.where(hit, disc, 0))
        r = np.where(hit & (d > 0.2) & (d < r), d, r)
    r[r > 8.0] = np.inf
    pts = np.column_stack((LIDAR_X + r * np.cos(BEAMS), r * np.sin(BEAMS)))
    return pts[np.isfinite(r)], 1.57


def collides(x, y, yaw, rocks):
    # eBot rectangle 0.62 x 0.49 against each circle
    for cx, cy, rad in rocks:
        lx = math.cos(-yaw) * (cx - x) - math.sin(-yaw) * (cy - y)
        ly = math.sin(-yaw) * (cx - x) + math.cos(-yaw) * (cy - y)
        qx, qy = max(-0.31, min(0.31, lx)), max(-0.245, min(0.245, ly))
        if math.hypot(lx - qx, ly - qy) < rad:
            return True
    return False


def run(route, rocks, start=(0.0, 0.0, 0.0), limit=400.0):
    x, y, yaw = start; idx = 0; dt = 0.05; tm = 0.0; reached = []
    while tm < limit:
        while idx < len(route):
            tol = t.final_tolerance_m if idx == len(route) - 1 else t.waypoint_tolerance_m
            if math.hypot(route[idx][0] - x, route[idx][1] - y) >= tol:
                break
            reached.append(idx); idx += 1
        if idx == len(route):
            return tm, reached
        tx, ty = route[idx]
        dist = math.hypot(tx - x, ty - y)
        gh = t.wrap(math.atan2(ty - y, tx - x) - yaw)
        pts, fov = scan(x, y, yaw, rocks)
        h, free = (gh, 0.0) if abs(gh) > fov else t.choose_heading(pts, gh, dist, fov)
        v, w = t.compute_cmd(h, free, dist, idx == len(route) - 1)
        assert abs(v) <= t.max_linear_mps + 1e-9 and abs(w) <= t.max_angular_rps + 1e-9
        w *= 0.85                       # skid steer: assume it under-turns a little
        x += v * math.cos(yaw) * dt; y += v * math.sin(yaw) * dt; yaw = t.wrap(yaw + w * dt)
        assert not collides(x, y, yaw, rocks), f'collision at ({x:.2f}, {y:.2f}) t={tm:.1f}'
        tm += dt
    raise AssertionError(f'timed out at waypoint {idx + 1}, pos ({x:.2f}, {y:.2f})')


if __name__ == '__main__':
    route = [(2.0, 0.0), (4.0, 0.0), (4.0, 3.0), (1.0, 3.0), (0.0, 1.0)]
    cases = {
        'clear route': [],
        'rock on each leg': [(1.0, 0.0, 0.25), (3.0, 0.05, 0.3), (4.0, 1.5, 0.3), (2.5, 3.0, 0.35)],
        'gate between two rocks': [(3.0, 0.75, 0.3), (3.0, -0.75, 0.3), (4.1, 1.5, 0.25)],
    }
    for name, rocks in cases.items():
        tm, reached = run(route, rocks)
        assert reached == list(range(len(route))), reached
        print(f'ok  {name:24s} finished in {tm:5.1f} s')
