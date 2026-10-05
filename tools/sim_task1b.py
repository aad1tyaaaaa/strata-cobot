#!/usr/bin/env python3
'''Offline check of the Task 1B control law on a kinematic UR7e (DH model, resolved-rate
IK) from the launch's initial joint pose. No ROS needed. Fails (assert) if a waypoint is
missed, the elbow passes straight, or joint speeds spike.   python3 tools/sim_task1b.py'''
import os, sys, types, importlib.util
import numpy as np
from scipy.spatial.transform import Rotation

HERE = os.path.dirname(os.path.abspath(__file__))
for m in ['rclpy', 'rclpy.node', 'rclpy.executors', 'control_msgs', 'control_msgs.msg', 'controller_manager_msgs',
          'controller_manager_msgs.srv', 'geometry_msgs', 'geometry_msgs.msg', 'sensor_msgs',
          'sensor_msgs.msg', 'std_msgs', 'std_msgs.msg']:
    sys.modules.setdefault(m, types.ModuleType(m))
sys.modules['rclpy.node'].Node = object
sys.modules['rclpy.executors'].ExternalShutdownException = Exception
for m, n in [('control_msgs.msg', 'JointJog'), ('controller_manager_msgs.srv', 'SwitchController'),
             ('geometry_msgs.msg', 'PoseStamped'), ('geometry_msgs.msg', 'TwistStamped'),
             ('sensor_msgs.msg', 'JointState'), ('std_msgs.msg', 'Int32')]:
    setattr(sys.modules[m], n, object)
spec = importlib.util.spec_from_file_location('t', os.path.join(HERE, '..', 'algorithms', 'scripts', 'task1b', 'task1b.py'))
t = importlib.util.module_from_spec(spec); spec.loader.exec_module(t)

# UR7e = UR5e kinematics (ur_description/config/ur7e/default_kinematics.yaml); last d
# includes tool_length 0.02041 (ur_macro.xacro). base_link coincides with the DH base here.
D = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996 + 0.02041]
A = [0, -0.425, -0.3922, 0, 0, 0]
AL = [np.pi / 2, 0, 0, np.pi / 2, -np.pi / 2, 0]


def fk(q):
    T = np.eye(4)
    for i in range(6):
        ct, st, ca, sa = np.cos(q[i]), np.sin(q[i]), np.cos(AL[i]), np.sin(AL[i])
        T = T @ np.array([[ct, -st * ca, st * sa, A[i] * ct], [st, ct * ca, -ct * sa, A[i] * st],
                          [0, sa, ca, D[i]], [0, 0, 0, 1]])
    return T


def jac(q, e=1e-6):
    T0 = fk(q); J = np.zeros((6, 6))
    for i in range(6):
        dq = np.array(q, float); dq[i] += e; T1 = fk(dq)
        J[:3, i] = (T1[:3, 3] - T0[:3, 3]) / e
        J[3:, i] = Rotation.from_matrix(T1[:3, :3] @ T0[:3, :3].T).as_rotvec() / e
    return J


if __name__ == '__main__':
    q = np.array([0, -0.523599, -2.443461, -0.174533, 1.570796, 1.570796])  # initial_positions.yaml
    dt, tm, hold, idx, prev = 0.05, 0.0, None, 0, np.zeros(3)
    min_elbow, max_qd, times = 9.0, 0.0, []
    while idx < len(t.waypoints) and tm < 300:
        T = fk(q)
        lin, ang, dist, ae = t.compute_twist(T[:3, 3], Rotation.from_matrix(T[:3, :3]).as_quat(),
                                             t.waypoints[idx], prev, dt)
        prev = lin
        assert np.linalg.norm(lin) <= t.cap_linear_mps and np.linalg.norm(ang) <= t.cap_angular_rps
        J = jac(q)
        qd = np.linalg.solve(J.T @ J + 1e-4 * np.eye(6), J.T @ np.r_[lin, ang])
        max_qd = max(max_qd, abs(qd).max())
        q = q + qd * dt; tm += dt
        min_elbow = min(min_elbow, abs(q[2]))
        if dist < t.pos_tolerance_m and ae < t.ang_tolerance_rad:
            hold = tm if hold is None else hold
            if tm - hold >= t.hold_time_s:
                times.append(round(tm, 1)); idx += 1; hold = None
        else:
            hold = None
    print(f'waypoints done at {times} s, min |elbow| {min_elbow:.3f} rad, max joint speed {max_qd:.2f} rad/s')
    assert idx == len(t.waypoints), f'stuck at waypoint {idx + 1}'
    assert min_elbow > 0.05, 'elbow went through straight (singular)'
    assert max_qd < 1.0, 'joint speed spike'
    print('ok')
