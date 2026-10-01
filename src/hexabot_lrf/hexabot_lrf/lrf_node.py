"""ROS 2 driver for the 3 km eye-safe laser rangefinder.

Topics   ~/range (sensor_msgs/Range), ~/measurement (LrfMeasurement),
         ~/status (LrfStatus, periodic self-check)
         subscribes ~/inhibit (std_msgs/Bool): no ranging while true
Services ~/measure (LrfMeasure), ~/configure (LrfConfigure),
         ~/continuous (std_srvs/SetBool)

With ``simulate:=true`` no serial port is opened: ranges come from a one-beam
LaserScan (the Gazebo LRF sensor, ``sim_scan_topic``) or are reported as out
of range when there is none.
"""
import threading
import time
from typing import List, Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from hexabot_interfaces.msg import LrfMeasurement, LrfStatus
from hexabot_interfaces.srv import LrfConfigure, LrfMeasure

from . import protocol as p

TARGET_NAMES = {'first': p.Target.FIRST, 'last': p.Target.LAST, 'multi': p.Target.MULTI}


class LrfNode(Node):
    def __init__(self) -> None:
        super().__init__('lrf')
        self.port = self.declare_parameter('port', '/dev/ttyUSB1').value
        self.baud = self.declare_parameter('baudrate', 115200).value
        self.frame_id = self.declare_parameter('frame_id', 'lrf_link').value
        self.simulate = self.declare_parameter('simulate', False).value
        self.sim_topic = self.declare_parameter('sim_scan_topic', 'lrf/scan').value
        target = self.declare_parameter('target_mode', 'first').value
        self.target = TARGET_NAMES.get(target, p.Target.FIRST)
        self.min_gate = self.declare_parameter('min_gate_m', 15).value
        self.frequency = self.declare_parameter('frequency_hz', 5).value
        self.min_interval = self.declare_parameter('min_interval', 0.1).value
        self.self_check_period = self.declare_parameter('self_check_period', 10.0).value
        self.timeout = self.declare_parameter('response_timeout', 1.0).value

        self.group = ReentrantCallbackGroup()
        self.range_pub = self.create_publisher(Range, '~/range', 10)
        self.meas_pub = self.create_publisher(LrfMeasurement, '~/measurement', 10)
        self.status_pub = self.create_publisher(LrfStatus, '~/status', 10)
        self.create_subscription(Bool, '~/inhibit', self._on_inhibit, 10, callback_group=self.group)
        self.create_service(LrfMeasure, '~/measure', self._srv_measure, callback_group=self.group)
        self.create_service(LrfConfigure, '~/configure', self._srv_configure, callback_group=self.group)
        self.create_service(SetBool, '~/continuous', self._srv_continuous, callback_group=self.group)

        self.inhibited = False
        self.continuous = False
        self.last_shot = 0.0
        self.lock = threading.Lock()
        self.result_event = threading.Event()
        self.pending: List[p.RangeResult] = []
        self.last_result: Optional[List[p.RangeResult]] = None
        self.status = LrfStatus()
        self.sim_range: Optional[float] = None
        self.serial = None
        self.parser = p.Parser()
        self.stop = threading.Event()

        if self.simulate:
            self.create_subscription(LaserScan, self.sim_topic, self._on_sim_scan, 10,
                                     callback_group=self.group)
            self.status.connected = True
            self.get_logger().info(f'simulated rangefinder (beam from {self.sim_topic})')
        else:
            self._open()
        self.create_timer(self.self_check_period, self._self_check, callback_group=self.group)
        self.create_timer(1.0 / max(1, self.frequency), self._sim_continuous, callback_group=self.group)

    # -- serial ----------------------------------------------------------------
    def _open(self) -> None:
        try:
            import serial  # noqa: PLC0415
            self.serial = serial.Serial(self.port, self.baud, timeout=0.05)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'cannot open {self.port}: {e}')
            return
        self.status.connected = True
        threading.Thread(target=self._reader, daemon=True).start()
        # Configure the module: target mode, minimum gate, continuous frequency.
        for frame in (p.stop_ranging(), p.set_target(self.target), p.set_min_gate(self.min_gate),
                      p.set_frequency(self.frequency), p.query(p.Cmd.MCU_VERSION)):
            self._send(frame)
            time.sleep(0.05)
        self.get_logger().info(f'rangefinder on {self.port} @ {self.baud}')

    def _send(self, frame: bytes) -> bool:
        if self.simulate:
            return True
        if self.serial is None:
            return False
        try:
            self.serial.write(frame)
            return True
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'serial write failed: {e}')
            return False

    def _reader(self) -> None:
        while not self.stop.is_set():
            try:
                data = self.serial.read(64)
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f'serial read failed: {e}')
                self.status.connected = False
                return
            if not data:
                self._flush_pending()
                continue
            for frame in self.parser.feed(data):
                self._on_frame(frame)

    def _on_frame(self, f: p.Frame) -> None:
        if f.cmd in (p.Cmd.SINGLE, p.Cmd.CONTINUOUS) and len(f.params) == 4:
            with self.lock:
                self.pending.append(p.decode_range(f.params))
                multi = self.target == p.Target.MULTI
            if not multi:
                self._flush_pending()
        elif f.cmd == p.Cmd.SELF_CHECK and len(f.params) == 4:
            s = p.decode_self_check(f.params)
            for k, v in s.items():
                setattr(self.status, k, v)
            self.status.header.stamp = self.get_clock().now().to_msg()
            self.status_pub.publish(self.status)
        elif f.cmd == p.Cmd.ANOMALY and len(f.params) == 4:
            bad = [k for k, v in p.decode_anomaly(f.params).items() if not v and k != 'echo']
            self.get_logger().warn(f'ranging anomaly: {", ".join(bad) or "no echo"}')
        elif f.cmd in (p.Cmd.MCU_VERSION, p.Cmd.FPGA_VERSION) and len(f.params) >= 3:
            self.status.firmware = p.decode_version(f.params)
        elif f.cmd == p.Cmd.TOTAL_SHOTS and len(f.params) == 3:
            self.status.total_shots = p.decode_counter(f.params)

    def _flush_pending(self) -> None:
        with self.lock:
            if not self.pending:
                return
            results, self.pending = self.pending, []
        self._publish(results)

    # -- results ---------------------------------------------------------------
    def _publish(self, results: List[p.RangeResult]) -> LrfMeasurement:
        now = self.get_clock().now().to_msg()
        ranges = p.collect_ranges(results)
        m = LrfMeasurement()
        m.header.stamp = now
        m.header.frame_id = self.frame_id
        m.target_mode = int(self.target)
        m.valid = ranges is not None
        m.ranges = [float(r) for r in (ranges or [])]
        m.statuses = [r.status for r in results]
        m.out_of_range = ranges is None
        m.front_target = any(r.front_target for r in results)
        m.rear_target = any(r.rear_target for r in results)
        self.meas_pub.publish(m)
        r = Range()
        r.header = m.header
        r.radiation_type = Range.INFRARED
        r.field_of_view = p.BEAM_DIVERGENCE_RAD
        r.min_range = p.MIN_RANGE_M
        r.max_range = p.MAX_RANGE_M
        r.range = m.ranges[0] if m.valid else float('inf')
        self.range_pub.publish(r)
        self.last_result = results
        self.result_event.set()
        return m

    def _simulated_results(self) -> List[p.RangeResult]:
        d = self.sim_range
        if d is None or d < p.MIN_RANGE_M or d > p.MAX_RANGE_M:
            return [p.decode_range(bytes([0x04, 0, 0, 0]))]
        return [p.decode_range(p.encode_range(d))]

    # -- callbacks -------------------------------------------------------------
    def _on_inhibit(self, msg: Bool) -> None:
        if msg.data and not self.inhibited:
            self.get_logger().info('ranging inhibited (near-field guard)')
        self.inhibited = msg.data
        if self.inhibited and self.continuous:
            self._send(p.stop_ranging())

    def _on_sim_scan(self, msg: LaserScan) -> None:
        vals = [v for v in msg.ranges if v == v and msg.range_min <= v <= msg.range_max]
        self.sim_range = min(vals) if vals else None

    def _sim_continuous(self) -> None:
        if self.simulate and self.continuous and not self.inhibited:
            self._publish(self._simulated_results())

    def _self_check(self) -> None:
        if self.simulate:
            self.status.header.stamp = self.get_clock().now().to_msg()
            self.status.fpga_ok = self.status.laser_output = self.status.temperature_ok = True
            self.status.power_5v6_ok = self.status.bias_ok = True
            self.status_pub.publish(self.status)
        elif not self.continuous:
            self._send(p.self_check())

    def _srv_measure(self, req: LrfMeasure.Request, res: LrfMeasure.Response):
        if self.inhibited:
            res.success = False
            res.message = 'inhibited: something is too close in front of the rangefinder'
            return res
        wait = self.min_interval - (time.monotonic() - self.last_shot)
        if wait > 0:
            time.sleep(wait)
        if req.target_mode in (1, 2, 3) and req.target_mode != self.target:
            self.target = p.Target(req.target_mode)
            self._send(p.set_target(self.target))
            time.sleep(0.05)
        self.last_shot = time.monotonic()
        if self.simulate:
            res.measurement = self._publish(self._simulated_results())
        else:
            self.result_event.clear()
            if not self._send(p.single_ranging()) or not self.result_event.wait(self.timeout):
                res.success = False
                res.message = 'no answer from the rangefinder'
                return res
            if self.target == p.Target.MULTI:
                time.sleep(0.06)  # let the remaining targets arrive
                self._flush_pending()
            res.measurement = LrfMeasurement()
            results = self.last_result or []
            ranges = p.collect_ranges(results)
            res.measurement.valid = ranges is not None
            res.measurement.ranges = [float(r) for r in (ranges or [])]
            res.measurement.out_of_range = ranges is None
            res.measurement.header.frame_id = self.frame_id
            res.measurement.header.stamp = self.get_clock().now().to_msg()
        res.success = True
        res.message = 'ok' if res.measurement.valid else 'out of range (no echo or closer than 15 m)'
        return res

    def _srv_configure(self, req: LrfConfigure.Request, res: LrfConfigure.Response):
        try:
            if req.target_mode:
                self.target = p.Target(req.target_mode)
                self._send(p.set_target(self.target))
            if req.min_gate_m:
                self._send(p.set_min_gate(req.min_gate_m))
            if req.max_gate_m:
                self._send(p.set_max_gate(req.max_gate_m))
            if req.frequency_hz:
                self.frequency = req.frequency_hz
                self._send(p.set_frequency(req.frequency_hz))
        except ValueError as e:
            res.success = False
            res.message = str(e)
            return res
        res.success = True
        res.message = 'configured'
        return res

    def _srv_continuous(self, req: SetBool.Request, res: SetBool.Response):
        if req.data and self.inhibited:
            res.success = False
            res.message = 'inhibited'
            return res
        self.continuous = req.data
        self._send(p.continuous_ranging() if req.data else p.stop_ranging())
        res.success = True
        res.message = 'continuous ranging ' + ('on' if req.data else 'off')
        return res

    def destroy_node(self):
        self.stop.set()
        if self.serial is not None:
            try:
                self.serial.write(p.stop_ranging())
                self.serial.close()
            except Exception:  # noqa: BLE001
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LrfNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
