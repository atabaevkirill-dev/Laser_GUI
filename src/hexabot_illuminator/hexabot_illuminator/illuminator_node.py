"""ROS 2 driver for the IR laser illuminator.

The device and its protocol are the ones the former Laser_GUI controlled:
TCP (192.168.0.7:20108 by default) or a serial port (9600 8N1).

Topics   ~/state (IlluminatorState, polled every ``poll_period``)
         subscribes ``vision/state`` (VisionState): with ``auto_night`` the
         illuminator follows the day/night decision of the vision manager
         subscribes ~/camera_hfov (std_msgs/Float32, degrees): with
         ``spot_follows_camera`` the spot angle tracks the camera field of view
Services ~/set (SetIlluminator), ~/power (std_srvs/SetBool)
"""
import queue
import socket
import threading
import time
from typing import Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32
from std_srvs.srv import SetBool

from hexabot_interfaces.msg import IlluminatorState, VisionState
from hexabot_interfaces.srv import SetIlluminator

from . import protocol as p


class Link:
    """Byte link to the device (TCP, serial or simulated)."""

    def __init__(self, node: Node, kind: str):
        self.node = node
        self.kind = kind
        self.sock: Optional[socket.socket] = None
        self.ser = None
        self.sim = p.State(power=False, brightness=200, motor=p.spot_to_motor(12.0), spot_deg=12.0, fan=False,
                           firmware='sim')
        self.sim_rx = bytearray()

    def open(self, host: str, port: int, serial_port: str, baud: int) -> bool:
        try:
            if self.kind == 'tcp':
                self.sock = socket.create_connection((host, port), timeout=2.0)
                self.sock.settimeout(0.2)
            elif self.kind == 'serial':
                import serial  # noqa: PLC0415
                self.ser = serial.Serial(serial_port, baud, timeout=0.2)
            return True
        except Exception as e:  # noqa: BLE001
            self.node.get_logger().error(f'cannot connect to the illuminator ({self.kind}): {e}')
            return False

    def send(self, data: bytes) -> None:
        if self.kind == 'tcp':
            self.sock.sendall(data)
        elif self.kind == 'serial':
            self.ser.write(data)
        else:
            self._simulate(data)

    def recv(self) -> bytes:
        if self.kind == 'tcp':
            try:
                return self.sock.recv(64)
            except socket.timeout:
                return b''
        if self.kind == 'serial':
            return self.ser.read(7)
        data, self.sim_rx = bytes(self.sim_rx), bytearray()
        return data

    def _simulate(self, data: bytes) -> None:
        cmd, d1, d2 = (data[2], data[3]), data[4], data[5]
        s = self.sim
        if cmd == p.POWER:
            s.power = d1 == 1
            s.fan = s.power
        elif cmd == p.BRIGHTNESS:
            s.brightness = d1
        elif cmd in (p.SPOT_ANGLE, p.MOTOR_POSITION):
            s.motor = d1 << 8 | d2
            s.spot_deg = p.SPOT_MAX_DEG - s.motor / p.MOTOR_MAX * (p.SPOT_MAX_DEG - p.SPOT_MIN_DEG)
        replies = {
            p.QUERY_POWER: (int(bool(s.power)), 0),
            p.QUERY_BRIGHTNESS: (s.brightness, 0),
            p.QUERY_MOTOR: (s.motor >> 8, s.motor & 0xFF),
            p.QUERY_FAN: (int(bool(s.fan)), 0),
            p.QUERY_SPOT: divmod(round(s.spot_deg * 100), 256),
        }
        if cmd in replies:
            self.sim_rx += p.packet(cmd, *replies[cmd])

    def close(self) -> None:
        for h in (self.sock, self.ser):
            try:
                if h is not None:
                    h.close()
            except Exception:  # noqa: BLE001
                pass


class IlluminatorNode(Node):
    def __init__(self) -> None:
        super().__init__('illuminator')
        kind = self.declare_parameter('transport', 'tcp').value       # tcp | serial | sim
        host = self.declare_parameter('host', '192.168.0.7').value
        tcp_port = self.declare_parameter('tcp_port', 20108).value
        serial_port = self.declare_parameter('serial_port', '/dev/ttyUSB2').value
        baud = self.declare_parameter('baudrate', 9600).value
        self.address = self.declare_parameter('address', 1).value
        self.frame_id = self.declare_parameter('frame_id', 'illuminator_link').value
        self.auto_night = self.declare_parameter('auto_night', True).value
        self.night_brightness = self.declare_parameter('night_brightness', 200).value
        self.follow_camera = self.declare_parameter('spot_follows_camera', True).value
        self.spot_margin = self.declare_parameter('spot_margin', 1.15).value
        poll = self.declare_parameter('poll_period', 1.0).value

        self.group = ReentrantCallbackGroup()
        self.state = p.State()
        self.connected = False
        self.parser = p.Parser()
        self.q: 'queue.Queue[bytes]' = queue.Queue()
        self.link = Link(self, kind)
        self.connected = self.link.open(host, tcp_port, serial_port, baud)
        self.state_pub = self.create_publisher(IlluminatorState, '~/state', 10)
        self.create_service(SetIlluminator, '~/set', self._srv_set, callback_group=self.group)
        self.create_service(SetBool, '~/power', self._srv_power, callback_group=self.group)
        self.create_subscription(VisionState, 'vision/state', self._on_vision, 10, callback_group=self.group)
        self.create_subscription(Float32, '~/camera_hfov', self._on_hfov, 10, callback_group=self.group)
        self.create_timer(poll, self._poll, callback_group=self.group)
        self.stop = threading.Event()
        threading.Thread(target=self._worker, daemon=True).start()
        self.last_night: Optional[bool] = None
        self.get_logger().info(f'illuminator via {kind} ({"connected" if self.connected else "offline"})')

    def _pkt(self, cmd, d1=0, d2=0) -> bytes:
        return p.packet(cmd, d1, d2, self.address)

    def _worker(self) -> None:
        """Send queued packets one at a time; wait for the reply of requests."""
        while not self.stop.is_set():
            try:
                pkt = self.q.get(timeout=0.2)
            except queue.Empty:
                continue
            if not self.connected:
                continue
            try:
                self.link.send(pkt)
                if (pkt[2], pkt[3]) in p.REQUESTS:
                    deadline = time.monotonic() + 0.5
                    got = False
                    while time.monotonic() < deadline and not got:
                        for reply in self.parser.feed(self.link.recv()):
                            p.apply(reply, self.state)
                            got = True
                else:
                    time.sleep(0.03)
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f'illuminator link error: {e}')
                self.connected = False

    def _poll(self) -> None:
        for q in (p.QUERY_POWER, p.QUERY_BRIGHTNESS, p.QUERY_SPOT, p.QUERY_FAN):
            self.q.put(self._pkt(q))
        self._publish()

    def _publish(self) -> IlluminatorState:
        s = IlluminatorState()
        s.header.stamp = self.get_clock().now().to_msg()
        s.header.frame_id = self.frame_id
        s.connected = self.connected
        s.power = bool(self.state.power)
        s.brightness = int(self.state.brightness or 0)
        s.spot_angle_deg = float(self.state.spot_deg or 0.0)
        s.motor_position = int(self.state.motor or 0)
        s.fan_on = bool(self.state.fan)
        s.firmware = self.state.firmware
        self.state_pub.publish(s)
        return s

    def _apply(self, power: bool, brightness: Optional[int], spot: Optional[float]) -> None:
        if brightness is not None:
            self.q.put(self._pkt(p.BRIGHTNESS, max(0, min(255, brightness))))
        if spot is not None and spot > 0:
            v = p.spot_to_motor(spot)
            self.q.put(self._pkt(p.SPOT_ANGLE, v >> 8, v & 0xFF))
        self.q.put(self._pkt(p.POWER, 1 if power else 0))
        for q in (p.QUERY_POWER, p.QUERY_BRIGHTNESS, p.QUERY_SPOT):
            self.q.put(self._pkt(q))

    def _srv_set(self, req: SetIlluminator.Request, res: SetIlluminator.Response):
        self._apply(req.power, req.brightness, req.spot_angle_deg if req.spot_angle_deg > 0 else None)
        time.sleep(0.3)
        res.success = self.connected
        res.message = 'sent' if self.connected else 'illuminator not connected'
        res.state = self._publish()
        return res

    def _srv_power(self, req: SetBool.Request, res: SetBool.Response):
        self._apply(req.data, self.night_brightness if req.data else None, None)
        res.success = self.connected
        res.message = ('on' if req.data else 'off') if self.connected else 'illuminator not connected'
        return res

    def _on_vision(self, msg: VisionState) -> None:
        if not self.auto_night or msg.night == self.last_night:
            return
        self.last_night = msg.night
        self.get_logger().info('night: illuminator on' if msg.night else 'day: illuminator off')
        self._apply(msg.night, self.night_brightness if msg.night else None, None)

    def _on_hfov(self, msg: Float32) -> None:
        if not self.follow_camera or not self.state.power:
            return
        target = max(p.SPOT_MIN_DEG, min(p.SPOT_MAX_DEG, msg.data * self.spot_margin))
        if self.state.spot_deg is None or abs(target - self.state.spot_deg) > 0.5:
            v = p.spot_to_motor(target)
            self.q.put(self._pkt(p.SPOT_ANGLE, v >> 8, v & 0xFF))
            self.state.spot_deg = target

    def destroy_node(self):
        self.stop.set()
        self.link.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IlluminatorNode()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
