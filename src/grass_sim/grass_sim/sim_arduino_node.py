"""
Simulated GRASS Arduino: the REAL firmware sketch behind a virtual serial port, plus a physical arm.

    grass_controller -> grass_serial -> [pty: link_path] -> this node -> real sketch (host build)
                                                                     -> /joint_states (RViz)

The sketch is compiled for the host by the GRASS repository's firmware/host_test harness, so the
protocol, configuration handshake, AccelStepper profiles, homing and switch protection are the
firmware's own code. Nothing here re-implements robot logic.

The arm is modelled the way a stepper driver sees it: every rising edge on an axis's STEP pin moves
the physical joint one step in the direction given by its DIR pin. The physical pose therefore
starts wherever the arm was left (initial_* parameters, not the firmware's step counter) and only
becomes equal to the firmware's belief after HOME, exactly as on the real robot. Each home switch
is closed when the physical joint is at or beyond its configured switch coordinate, so homing finds
the switch where it physically is and ordinary moves into a switch produce LIMIT.
"""

import errno
import math
import os
import pty
import sys
import time
import tty

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

AXES = ('j1', 'z', 'j2')            # firmware axis order (0, 1, 2)
JOINT_NAMES = ('j1', 'z', 'j2')     # URDF joint names


class SimArduino(Node):

    def __init__(self):
        super().__init__('sim_arduino')
        self.declare_parameter('grass_repo', '')
        self.declare_parameter('link_path', '/tmp/grass_sim_arduino')
        self.declare_parameter('initial_j1_deg', 115.0)
        self.declare_parameter('initial_z_mm', 75.0)
        self.declare_parameter('initial_j2_deg', -155.0)
        self.declare_parameter('tick_s', 0.02)
        # The DIR level is sampled once per chunk. 5 ms is shorter than the first step after any
        # stop or reversal, so a chunk never holds steps in both directions.
        self.declare_parameter('chunk_ms', 5)
        self.declare_parameter('time_scale', 1.0)

        repo = self.get_parameter('grass_repo').value
        host_test = os.path.join(repo, 'firmware', 'host_test')
        if not os.path.isfile(os.path.join(host_test, 'host_device.py')):
            raise RuntimeError(f'grass_repo={repo!r} has no firmware/host_test/host_device.py')
        sys.path.insert(0, host_test)
        from host_device import HostDevice

        self._chunk_ms = int(self.get_parameter('chunk_ms').value)
        self._time_scale = float(self.get_parameter('time_scale').value)
        self._phys = [float(self.get_parameter(f'initial_{n}').value)
                      for n in ('j1_deg', 'z_mm', 'j2_deg')]
        self._edges = [None] * 3
        self._step_pins = [None] * 3
        self._forced = [None] * 3

        self.get_logger().info('building and booting the host firmware...')
        self._device = HostDevice()

        self._master, self._slave = pty.openpty()
        tty.setraw(self._master)
        tty.setraw(self._slave)
        os.set_blocking(self._master, False)
        self._link = self.get_parameter('link_path').value
        if os.path.lexists(self._link):
            os.remove(self._link)
        os.symlink(os.ttyname(self._slave), self._link)
        self._rx = b''
        self._write(self._device.boot)

        self._pub = self.create_publisher(JointState, 'joint_states', 10)
        self._last = time.monotonic()
        self._carry_ms = 0.0
        self.create_timer(float(self.get_parameter('tick_s').value), self._tick)
        self.get_logger().info(
            f'simulated Arduino on {self._link} -> {os.ttyname(self._slave)}; physical arm at '
            f'j1={self._phys[0]:g} deg z={self._phys[1]:g} mm j2={self._phys[2]:g} deg')

    def _write(self, lines):
        for line in lines:
            try:
                os.write(self._master, (line + '\n').encode())
            except OSError as e:
                if e.errno != errno.EAGAIN:     # nobody reading the port yet: drop, like a UART
                    raise

    def _read_serial(self):
        while True:
            try:
                data = os.read(self._master, 256)
            except OSError as e:
                if e.errno in (errno.EAGAIN, errno.EIO):
                    return
                raise
            if not data:
                return
            self._rx += data
            while b'\n' in self._rx:
                raw, _, self._rx = self._rx.partition(b'\n')
                line = raw.decode(errors='replace').strip()
                if line:
                    self._write(self._device.send(line))

    def _tick(self):
        self._read_serial()
        now = time.monotonic()
        self._carry_ms += min((now - self._last) * 1000.0 * self._time_scale, 200.0)
        self._last = now
        while self._carry_ms >= 1.0:
            ms = min(self._chunk_ms, int(self._carry_ms))
            self._carry_ms -= ms
            self._write(self._device.advance(ms))
            self._update_arm(self._device.state())
        self._publish()

    def _update_arm(self, s):
        if s.get('built') != '1':
            return
        for a, n in enumerate(AXES):
            step_pin, dir_pin = s[f'{n}.step'], s[f'{n}.dir']
            steps_per_unit = float(s[f'{n}.steps'])
            edges = int(s[f'p{step_pin}'].split(':')[2])
            if self._edges[a] is not None and self._step_pins[a] == step_pin:
                delta = edges - self._edges[a] if edges >= self._edges[a] else edges
                if delta:
                    forward = int(s[f'p{dir_pin}'].split(':')[1]) ^ int(s[f'{n}.dinv'])
                    self._phys[a] += (delta if forward else -delta) / steps_per_unit
            self._edges[a], self._step_pins[a] = edges, step_pin

            beyond = (self._phys[a] - float(s[f'{n}.switch'])) * int(s[f'{n}.hdir'])
            active = beyond >= -1e-9
            if active != self._forced[a]:
                self._forced[a] = active
                self._write(self._device.force_switch(a, active))

    def _publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = [math.radians(self._phys[0]), self._phys[1] / 1000.0,
                        math.radians(self._phys[2])]
        self._pub.publish(msg)

    def close(self):
        self._device.close()
        if os.path.islink(self._link):
            os.remove(self._link)
        for fd in (self._master, self._slave):
            try:
                os.close(fd)
            except OSError:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = SimArduino()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
