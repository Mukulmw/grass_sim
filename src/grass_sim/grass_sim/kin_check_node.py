"""
Kinematics validation in RViz, against grass_controller.kinematics (the controller's own code).

FK check: the stick model is posed by robot_state_publisher from /joint_states (sliders), i.e. by
the URDF/TF chain. For the same joint values this node evaluates kinematics.forward_kinematics()
and draws its result as a translucent sphere around the stick tip; the TF-vs-FK error in mm is in
the table window (top right of the screen, always on top).
The TF is matched to the exact joint_states message it was computed from (by stamp).

IK check: drag the orange target. Both elbow solutions of kinematics.inverse_kinematics() are drawn
as ghost sticks (green: inside robot.yaml limits, red: outside); the table lists their joint values,
the elbow pick_ik_solution() would choose and the round-trip error |FK(IK(target)) - target|.
"""

import math
import signal
import sys

import numpy as np
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtGui import QColor, QFont
from python_qt_binding.QtWidgets import (QApplication, QHeaderView, QTableWidget,
                                         QTableWidgetItem)
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Point
from interactive_markers import InteractiveMarkerServer
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import (InteractiveMarker, InteractiveMarkerControl,
                                    InteractiveMarkerFeedback, Marker, MarkerArray)

from grass_controller import kinematics
from grass_controller.robot_config import default_robot_config_path, load_robot_config

GREEN, RED, WHITE = (0.2, 0.9, 0.3), (0.95, 0.2, 0.2), (1.0, 1.0, 1.0)


def rgba(rgb, a=1.0):
    return ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=a)


def pt(x_mm, y_mm, z_mm):
    return Point(x=x_mm / 1000.0, y=y_mm / 1000.0, z=z_mm / 1000.0)


class KinCheck(Node):

    def __init__(self):
        super().__init__('kin_check')
        self.declare_parameter('robot_config', '')
        self.declare_parameter('target_mm', [-150.0, 270.0, 50.0])
        self._cfg = load_robot_config(self.get_parameter('robot_config').value
                                      or default_robot_config_path())
        self._geo = {'l1_mm': self._cfg.l1_mm, 'l2_mm': self._cfg.l2_mm}
        self._target = list(self.get_parameter('target_mm').value)

        self._states = {}
        self._fk_markers = []
        self._tf = Buffer()
        TransformListener(self._tf, self)
        self._pub = self.create_publisher(MarkerArray, 'kin_check/markers', 10)
        self.create_subscription(JointState, 'joint_states', self._on_joints, 10)

        self._server = InteractiveMarkerServer(self, 'kin_check/ik_target')
        self._server.insert(self._target_marker(), feedback_callback=self._on_target)
        self._server.applyChanges()
        self._workspace = self._workspace_markers()
        self._ik_markers = self._solve_ik(log=True)
        self.create_timer(0.1, self._check_fk)

    # ------------------------------------------------------------------ helpers
    def _marker(self, ns, mid, mtype):
        m = Marker()
        m.header.frame_id = 'world'
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def _text(self, ns, mid, where, text, color=WHITE, size=0.022):
        m = self._marker(ns, mid, Marker.TEXT_VIEW_FACING)
        m.pose.position = where
        m.scale.z = size
        m.color = rgba(color)
        m.text = text
        return m

    def _in_limits(self, j):
        c = self._cfg
        return {'j1': c.j1.minimum <= j.j1_deg <= c.j1.maximum,
                'z': c.z.minimum <= j.z_joint_mm <= c.z.maximum,
                'j2': c.j2.minimum <= j.j2_deg <= c.j2.maximum}

    def _publish(self):
        self._pub.publish(MarkerArray(markers=self._workspace + self._fk_markers
                                      + self._ik_markers))

    # ------------------------------------------------------------------ workspace
    def _workspace_markers(self):
        c, out = self._cfg, []
        for i, r in enumerate((abs(c.l1_mm - c.l2_mm), c.l1_mm + c.l2_mm)):
            ring = self._marker('workspace_rings', i, Marker.LINE_STRIP)
            ring.scale.x = 0.0015
            ring.color = rgba((0.6, 0.6, 0.6), 0.6)
            ring.points = [pt(r * math.cos(t), r * math.sin(t), 0)
                           for t in np.linspace(0, 2 * math.pi, 181)]
            out.append(ring)
        ws = self._marker('workspace_limits', 0, Marker.POINTS)
        ws.scale.x = ws.scale.y = 0.003
        ws.color = rgba((0.35, 0.55, 0.95), 0.35)
        t1, t2 = np.meshgrid(np.radians(np.linspace(c.j1.minimum, c.j1.maximum, 90)),
                             np.radians(np.linspace(c.j2.minimum, c.j2.maximum, 90)))
        xs = -(c.l1_mm * np.sin(t1) + c.l2_mm * np.sin(t1 + t2))
        ys = c.l1_mm * np.cos(t1) + c.l2_mm * np.cos(t1 + t2)
        ws.points = [pt(float(x), float(y), 0) for x, y in zip(xs.ravel(), ys.ravel())]
        out.append(ws)
        for i, (label, x, y) in enumerate((('+Y (j1=j2=0)', 0, 660), ('-X (+angle)', -660, 0),
                                           ('+X', 660, 0))):
            out.append(self._text('axes', i, pt(x, y, 0), label, (0.7, 0.7, 0.7), 0.025))
        return out

    # ------------------------------------------------------------------ FK check
    def _on_joints(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        if not all(n in idx for n in ('j1', 'z', 'j2')):
            return
        stamp = Time.from_msg(msg.header.stamp).nanoseconds
        self._states[stamp] = (math.degrees(msg.position[idx['j1']]),
                               msg.position[idx['z']] * 1000.0,
                               math.degrees(msg.position[idx['j2']]))
        while len(self._states) > 100:
            del self._states[min(self._states)]

    def _check_fk(self):
        try:
            tf = self._tf.lookup_transform('world', 'camera_link', Time())
        except TransformException:
            return
        joints = self._states.get(Time.from_msg(tf.header.stamp).nanoseconds)
        if joints is None:
            return
        fk = kinematics.forward_kinematics(*joints, **self._geo)
        t = tf.transform.translation
        tf_mm = (t.x * 1000, t.y * 1000, t.z * 1000)
        err = math.dist(tf_mm, (fk.x_mm, fk.y_mm, fk.z_cart_mm))

        sphere = self._marker('fk', 0, Marker.SPHERE)
        sphere.pose.position = pt(fk.x_mm, fk.y_mm, fk.z_cart_mm)
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.03
        sphere.color = rgba(GREEN if err < 0.01 else RED, 0.35)
        elbow = kinematics.elbow_of(joints[2]) or 'singular'
        self.fk_rows = [
            ('Sliders → kinematics.py', joints, (fk.x_mm, fk.y_mm, fk.z_cart_mm),
             f'error {err:.4f} mm', err < 0.01),
            ('URDF / TF', None, tf_mm, f'elbow {elbow}', None),
        ]
        self._fk_markers = [sphere]
        self._publish()

    # ------------------------------------------------------------------ IK check
    def _target_marker(self):
        im = InteractiveMarker()
        im.header.frame_id = 'world'
        im.name = 'ik_target'
        im.description = 'IK target (drag)'
        im.scale = 0.12
        im.pose.position = pt(*self._target)
        im.pose.orientation.w = 1.0
        ball = Marker()
        ball.type = Marker.SPHERE
        ball.scale.x = ball.scale.y = ball.scale.z = 0.02
        ball.color = rgba((1.0, 0.55, 0.0))
        body = InteractiveMarkerControl()
        body.always_visible = True
        body.markers.append(ball)
        body.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        body.orientation.w, body.orientation.y = math.sqrt(0.5), math.sqrt(0.5)  # normal = Z
        im.controls.append(body)
        for name, (qx, qy, qz) in (('move_x', (1, 0, 0)), ('move_y', (0, 0, 1)),
                                   ('move_z', (0, 1, 0))):
            c = InteractiveMarkerControl()
            c.name = name
            h = math.sqrt(0.5)
            c.orientation.w, c.orientation.x, c.orientation.y, c.orientation.z = h, h * qx, h * qy, h * qz
            c.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            im.controls.append(c)
        return im

    def _on_target(self, fb):
        p = fb.pose.position
        self._target = [p.x * 1000, p.y * 1000, p.z * 1000]
        self._ik_markers = self._solve_ik(log=fb.event_type == InteractiveMarkerFeedback.MOUSE_UP)
        self._publish()

    def _solve_ik(self, log=False):
        x, y, z = self._target
        out, lines = [], [f'IK CHECK  target x {x:.1f}  y {y:.1f}  z {z:.1f} mm']
        rows = [('IK target (drag)', None, (x, y, z), '', None)]
        try:
            picked = kinematics.pick_ik_solution(
                x, y, z, **self._geo,
                j1_limits_deg=(self._cfg.j1.minimum, self._cfg.j1.maximum),
                j2_limits_deg=(self._cfg.j2.minimum, self._cfg.j2.maximum))[1]
        except ValueError:
            picked = None
        for i, elbow in enumerate(('down', 'up')):
            try:
                j = kinematics.inverse_kinematics(x, y, z, elbow=elbow, **self._geo)
            except ValueError as e:
                lines.append(f'{elbow:>4}: {e}')
                rows.append((f'IK {elbow}', None, None, 'unreachable', False))
                continue
            ok = self._in_limits(j)
            back = kinematics.forward_kinematics(j.j1_deg, j.z_joint_mm, j.j2_deg, **self._geo)
            rt = math.dist((back.x_mm, back.y_mm, back.z_cart_mm), (x, y, z))
            a1 = math.radians(j.j1_deg)
            ex, ey = -self._cfg.l1_mm * math.sin(a1), self._cfg.l1_mm * math.cos(a1)

            stick = self._marker('ik_solution', i, Marker.LINE_STRIP)
            stick.scale.x = 0.005
            stick.color = rgba(GREEN if all(ok.values()) else RED, 0.55)
            stick.points = [pt(0, 0, 0), pt(0, 0, z), pt(ex, ey, z), pt(x, y, z)]
            out.append(stick)
            out.append(self._text('ik_label', i, pt(ex, ey, z + 30), elbow,
                                  GREEN if all(ok.values()) else RED))
            bad = [k for k, v in ok.items() if not v]
            lines.append(f'{elbow:>4}: j1 {j.j1_deg:8.2f}  z {j.z_joint_mm:6.1f}  '
                         f'j2 {j.j2_deg:8.2f}  round-trip {rt:.2e} mm  '
                         + ('in limits' if not bad else 'OUT: ' + ','.join(bad))
                         + ('  <- picked' if elbow == picked else ''))
            rows.append((f'IK {elbow}' + ('  ← picked' if elbow == picked else ''),
                         (j.j1_deg, j.z_joint_mm, j.j2_deg),
                         (back.x_mm, back.y_mm, back.z_cart_mm),
                         ('in limits' if not bad else 'OUT: ' + ', '.join(bad))
                         + f'   round-trip {rt:.1e} mm', not bad))
        if picked is None and len(lines) == 3:
            lines.append('pick_ik_solution: no elbow inside the j1/j2 limits')
        self.ik_rows = rows
        if log:
            self.get_logger().info('\n'.join(lines))
        return out


class Readout(QTableWidget):
    """Always-on-top table at the top right of the screen with the FK/IK numbers."""

    HEADERS = ('', 'J1 [deg]', 'Z [mm]', 'J2 [deg]', 'X [mm]', 'Y [mm]', 'Z [mm]', 'check')

    def __init__(self, node, font_pt):
        super().__init__(5, len(self.HEADERS))
        self._node = node
        self.setWindowTitle('GRASS kinematics check')
        self.setWindowFlags(self.windowFlags() | Qt.WindowStaysOnTopHint)
        font = QFont('Monospace', font_pt)
        font.setStyleHint(QFont.TypeWriter)
        self.setFont(font)
        self.horizontalHeader().setFont(QFont('Sans', font_pt, QFont.Bold))
        self.setHorizontalHeaderLabels(self.HEADERS)
        self.verticalHeader().hide()
        self.setEditTriggers(QTableWidget.NoEditTriggers)
        self.setSelectionMode(QTableWidget.NoSelection)
        self.setStyleSheet('QTableWidget { background: #1e1e22; color: #eeeeee; gridline-color: #444; }'
                           'QHeaderView::section { background: #2d2d33; color: #eeeeee; }')
        self.refresh()
        self.resizeColumnsToContents()
        self.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        geo = QApplication.primaryScreen().availableGeometry()
        width = min(geo.width() - 40, self.horizontalHeader().length() + 60)
        self.resize(width, self.verticalHeader().length() + self.horizontalHeader().height() + 12)
        self.move(geo.right() - self.width() - 20, geo.top() + 40)

    def _set(self, row, col, text, color=None, bold=False):
        item = QTableWidgetItem(text)
        item.setTextAlignment(int(Qt.AlignRight | Qt.AlignVCenter) if 0 < col < 7
                              else int(Qt.AlignLeft | Qt.AlignVCenter))
        if color:
            item.setForeground(QColor(color))
        if bold:
            f = self.font()
            f.setBold(True)
            item.setFont(f)
        self.setItem(row, col, item)

    def refresh(self):
        rows = (getattr(self._node, 'fk_rows', None) or [('Sliders → kinematics.py', None, None,
                                                         'waiting for TF', None)])
        rows = list(rows) + [None] * (2 - len(rows)) + list(self._node.ik_rows)
        for r in range(self.rowCount()):
            for c in range(self.columnCount()):
                self._set(r, c, '')
        for r, row in enumerate(rows[:self.rowCount()]):
            if row is None:
                continue
            name, joints, xyz, check, ok = row
            self._set(r, 0, name, bold=True)
            if joints:
                for c, v in enumerate(joints, 1):
                    self._set(r, c, f'{v:.2f}' if c != 2 else f'{v:.1f}', '#8fd3ff')
            if xyz:
                for c, v in enumerate(xyz, 4):
                    self._set(r, c, f'{v:.2f}')
            self._set(r, 7, check, None if ok is None else ('#55dd66' if ok else '#ff5555'))


def main(args=None):
    rclpy.init(args=args)
    node = KinCheck()
    node.declare_parameter('font_pt', 16)
    app = QApplication(sys.argv)
    table = Readout(node, int(node.get_parameter('font_pt').value))
    table.show()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def pump():
        if not rclpy.ok():          # ROS already shut down by Ctrl-C / launch
            app.quit()
            return
        try:
            for _ in range(50):
                executor.spin_once(timeout_sec=0)
        except Exception:           # context invalidated mid-spin during shutdown
            app.quit()
            return
        table.refresh()

    timer = QTimer()
    timer.timeout.connect(pump)
    timer.start(50)
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    try:
        app.exec_()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
