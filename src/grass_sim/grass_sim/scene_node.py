"""
RViz scene for the GRASS simulation: tray, joint-limited workspace, plants and routines.

Geometry and limits come from the controller's robot.yaml (through grass_controller.robot_config)
and the plants/routines from the same plants.yaml the controller uses, re-read when it changes.
Reachability uses grass_controller.kinematics, so a plant drawn red is one the controller would
also refuse. Plants are coloured by what the camera node reports: yellow while CAPTURE is
outstanding, blue once IMAGE_SAVED, red on IMAGE_ERROR.
"""

import math
import os

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, String
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
import yaml

from grass_controller import kinematics
from grass_controller.robot_config import default_robot_config_path, load_robot_config

# Tray of Kinematic_plot.py (mm): 400 x 600, starting 70 mm in front of the base axis.
TRAY_W, TRAY_H, TRAY_Y0 = 400.0, 600.0, 70.0
# D435 colour field of view.
HFOV, VFOV = math.radians(69.0), math.radians(42.0)

COLORS = {
    'idle': (0.30, 0.75, 0.35), 'capturing': (1.0, 0.85, 0.10), 'saved': (0.25, 0.55, 1.0),
    'error': (0.90, 0.15, 0.15), 'unreachable': (0.90, 0.15, 0.15),
}


def rgba(rgb, a=1.0):
    return ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=a)


class Scene(Node):

    def __init__(self):
        super().__init__('grass_scene')
        self.declare_parameter('plants_file', '')
        self.declare_parameter('robot_config', '')
        self.declare_parameter('tray_z_m', -0.30)
        self.declare_parameter('frustum_depth_m', 0.30)
        self._plants_file = self.get_parameter('plants_file').value
        self._tray_z = float(self.get_parameter('tray_z_m').value)
        self._frustum = float(self.get_parameter('frustum_depth_m').value)
        config_path = self.get_parameter('robot_config').value or default_robot_config_path()
        self._cfg = load_robot_config(config_path)

        self._mtime = None
        self._plants = {}
        self._routines = {}
        self._status = {}
        self._static = self._static_markers()

        self._pub = self.create_publisher(MarkerArray, 'grass_sim/scene', 10)
        self.create_subscription(String, 'camera/capture', self._on_capture, 10)
        self.create_subscription(String, 'camera/status', self._on_camera, 10)
        self.create_timer(0.5, self._publish)

    def _marker(self, ns, mid, mtype, frame='world'):
        m = Marker()
        m.header.frame_id = frame
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def _static_markers(self):
        tray = self._marker('tray', 0, Marker.CUBE)
        tray.pose.position.y = (TRAY_Y0 + TRAY_H / 2) / 1000.0
        tray.pose.position.z = self._tray_z - 0.005
        tray.scale.x, tray.scale.y, tray.scale.z = TRAY_W / 1000, TRAY_H / 1000, 0.01
        tray.color = rgba((0.45, 0.33, 0.22), 0.9)

        ws = self._marker('workspace', 0, Marker.POINTS)
        ws.scale.x = ws.scale.y = 0.004
        ws.color = rgba((0.35, 0.55, 0.95), 0.35)
        j1 = np.radians(np.linspace(self._cfg.j1.minimum, self._cfg.j1.maximum, 80))
        j2 = np.radians(np.linspace(self._cfg.j2.minimum, self._cfg.j2.maximum, 80))
        t1, t2 = np.meshgrid(j1, j2)
        xs = -(self._cfg.l1_mm * np.sin(t1) + self._cfg.l2_mm * np.sin(t1 + t2)) / 1000
        ys = (self._cfg.l1_mm * np.cos(t1) + self._cfg.l2_mm * np.cos(t1 + t2)) / 1000
        ws.points = [Point(x=float(x), y=float(y), z=self._tray_z + 0.001)
                     for x, y in zip(xs.ravel(), ys.ravel())]

        frustum = self._marker('camera_fov', 0, Marker.LINE_LIST, frame='camera_optical_frame')
        frustum.frame_locked = True
        frustum.scale.x = 0.002
        frustum.color = rgba((1.0, 1.0, 1.0), 0.6)
        hx, hy = self._frustum * math.tan(HFOV / 2), self._frustum * math.tan(VFOV / 2)
        corners = [Point(x=sx * hx, y=sy * hy, z=self._frustum)
                   for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        for i, c in enumerate(corners):
            frustum.points += [Point(), c, c, corners[(i + 1) % 4]]
        return [tray, ws, frustum]

    def _reload(self):
        try:
            mtime = os.stat(self._plants_file).st_mtime
        except OSError:
            return
        if mtime == self._mtime:
            return
        self._mtime = mtime
        try:
            with open(self._plants_file) as f:
                data = yaml.safe_load(f) or {}
            self._plants = {int(k): v for k, v in (data.get('plants') or {}).items()}
            self._routines = data.get('routines') or {}
        except (OSError, yaml.YAMLError, ValueError, AttributeError) as e:
            self.get_logger().warning(f'cannot read {self._plants_file}: {e}')
            return
        self._status = {}
        self.get_logger().info(
            f'{len(self._plants)} plants, routines {list(self._routines)} from {self._plants_file}')

    def _reachable(self, p):
        try:
            if p.get('elbow'):
                j = kinematics.inverse_kinematics(
                    p['x_mm'], p['y_mm'], p['z_mm'], elbow=p['elbow'],
                    l1_mm=self._cfg.l1_mm, l2_mm=self._cfg.l2_mm)
            else:
                j, _ = kinematics.pick_ik_solution(
                    p['x_mm'], p['y_mm'], p['z_mm'], l1_mm=self._cfg.l1_mm, l2_mm=self._cfg.l2_mm,
                    j1_limits_deg=(self._cfg.j1.minimum, self._cfg.j1.maximum),
                    j2_limits_deg=(self._cfg.j2.minimum, self._cfg.j2.maximum))
        except (ValueError, KeyError, TypeError):
            return False
        return (self._cfg.j1.minimum <= j.j1_deg <= self._cfg.j1.maximum
                and self._cfg.j2.minimum <= j.j2_deg <= self._cfg.j2.maximum
                and self._cfg.z.minimum <= j.z_joint_mm <= self._cfg.z.maximum)

    def _plant_markers(self):
        out = []
        for pid, p in sorted(self._plants.items()):
            x, y, z = p['x_mm'] / 1000, p['y_mm'] / 1000, p['z_mm'] / 1000
            state = self._status.get(pid, 'idle' if self._reachable(p) else 'unreachable')

            plant = self._marker('plants', pid, Marker.CYLINDER)
            plant.pose.position.x, plant.pose.position.y = x, y
            plant.pose.position.z = self._tray_z + 0.03
            plant.scale.x = plant.scale.y = 0.05
            plant.scale.z = 0.06
            plant.color = rgba(COLORS[state])

            target = self._marker('targets', pid, Marker.SPHERE)
            target.pose.position.x, target.pose.position.y, target.pose.position.z = x, y, z
            target.scale.x = target.scale.y = target.scale.z = 0.012
            target.color = rgba(COLORS[state], 0.8)

            drop = self._marker('drop', pid, Marker.LINE_LIST)
            drop.scale.x = 0.001
            drop.color = rgba((0.8, 0.8, 0.8), 0.4)
            drop.points = [Point(x=x, y=y, z=z), Point(x=x, y=y, z=self._tray_z + 0.06)]

            label = self._marker('labels', pid, Marker.TEXT_VIEW_FACING)
            label.pose.position.x, label.pose.position.y = x, y
            label.pose.position.z = self._tray_z + 0.10
            label.scale.z = 0.03
            label.color = rgba((1.0, 1.0, 1.0))
            label.text = f'P{pid}' + (f' ({p["elbow"]})' if p.get('elbow') else '')
            out += [plant, target, drop, label]

        for i, (name, routine) in enumerate(self._routines.items()):
            path = self._marker('routine', i, Marker.LINE_STRIP)
            path.scale.x = 0.002
            path.color = rgba((1.0, 0.6, 0.2), 0.7)
            for pid in routine.get('plants') or []:
                p = self._plants.get(int(pid))
                if p:
                    path.points.append(Point(x=p['x_mm'] / 1000, y=p['y_mm'] / 1000,
                                             z=p['z_mm'] / 1000))
            out.append(path)
        return out

    def _on_capture(self, msg):
        parts = msg.data.split()
        if len(parts) == 2 and parts[0] == 'CAPTURE' and parts[1].isdigit():
            self._status[int(parts[1])] = 'capturing'

    def _on_camera(self, msg):
        parts = msg.data.split()
        if len(parts) >= 2 and parts[1].isdigit():
            if parts[0] == 'IMAGE_SAVED':
                self._status[int(parts[1])] = 'saved'
            elif parts[0] == 'IMAGE_ERROR':
                self._status[int(parts[1])] = 'error'

    def _publish(self):
        self._reload()
        clear = Marker()
        clear.action = Marker.DELETEALL
        self._pub.publish(MarkerArray(markers=[clear] + self._static + self._plant_markers()))


def main(args=None):
    rclpy.init(args=args)
    node = Scene()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
