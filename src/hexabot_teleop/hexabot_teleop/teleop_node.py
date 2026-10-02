"""Gamepad teleoperation (see config/gamepad.yaml for the button layout)."""
import rclpy
from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool

from hexabot_interfaces.msg import BodyPose, LocomotionState
from hexabot_interfaces.srv import LrfMeasure, SelectTarget, SetGait, SetMode

from .mapping import Mapper, next_gait

PARAMS = {
    'axis_forward': 1, 'axis_sideways': 0, 'axis_turn': 2, 'axis_pitch': 3,
    'button_deadman': 9, 'button_fast': 10, 'button_stand': 0, 'button_crouch': 1,
    'button_gait': 2, 'button_illuminator': 3, 'button_measure': 6, 'button_next_target': 4,
    'button_dpad_up': 11, 'button_dpad_down': 12, 'button_dpad_left': 13, 'button_dpad_right': 14,
    'max_forward': 0.12, 'max_sideways': 0.08, 'max_turn': 0.6, 'fast_factor': 1.6,
    'max_pitch': 0.2, 'head_rate': 0.8,
}


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop')
        cfg = {k: self.declare_parameter(k, v).value for k, v in PARAMS.items()}
        self.mapper = Mapper(cfg)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pose_pub = self.create_publisher(BodyPose, 'body_pose', 10)
        self.head_pub = self.create_publisher(Vector3, 'head/manual_rate', 10)
        self.create_subscription(Joy, 'joy', self.on_joy, 10)
        self.create_subscription(LocomotionState, 'locomotion/state', self.on_state, 10)
        self.mode_cli = self.create_client(SetMode, 'locomotion/set_mode')
        self.gait_cli = self.create_client(SetGait, 'locomotion/set_gait')
        self.ill_cli = self.create_client(SetBool, 'illuminator/power')
        self.lrf_cli = self.create_client(LrfMeasure, 'lrf/measure')
        self.target_cli = self.create_client(SelectTarget, 'targeting/select_target')
        self.state = None
        self.illuminator_on = False
        self.moving = False

    def on_state(self, msg):
        self.state = msg

    def call(self, client, req, what):
        if not client.service_is_ready():
            self.get_logger().warn(f'{what}: service {client.srv_name} not available')
            return
        client.call_async(req).add_done_callback(
            lambda f: self.get_logger().info(f'{what}: {getattr(f.result(), "message", "done")}'))

    def on_joy(self, msg):
        cmd = self.mapper.update(list(msg.axes), list(msg.buttons))
        moving = any(abs(v) > 1e-3 for v in (cmd.vx, cmd.vy, cmd.wz))
        if moving or self.moving:   # one zero after release, then stay quiet for Nav2
            t = Twist()
            t.linear.x, t.linear.y, t.angular.z = cmd.vx, cmd.vy, cmd.wz
            self.cmd_pub.publish(t)
        self.moving = moving
        pose = BodyPose()
        pose.pitch = cmd.pitch
        self.pose_pub.publish(pose)
        if cmd.pan_rate or cmd.tilt_rate:
            self.head_pub.publish(Vector3(x=cmd.pan_rate, y=cmd.tilt_rate, z=0.0))
        for action in cmd.actions:
            if action == 'stand_sit':
                standing = self.state is not None and self.state.mode == 'standing'
                self.call(self.mode_cli, SetMode.Request(mode='sit' if standing else 'stand'), 'mode')
            elif action == 'crouch':
                crouched = self.state is not None and self.state.crouched
                self.call(self.mode_cli, SetMode.Request(mode='normal' if crouched else 'crouch'), 'crouch')
            elif action == 'next_gait':
                current = self.state.gait if self.state else None
                self.call(self.gait_cli, SetGait.Request(gait=next_gait(current)), 'gait')
            elif action == 'illuminator':
                self.illuminator_on = not self.illuminator_on
                self.call(self.ill_cli, SetBool.Request(data=self.illuminator_on), 'illuminator')
            elif action == 'measure':
                self.call(self.lrf_cli, LrfMeasure.Request(target_mode=0), 'rangefinder')
            elif action == 'next_target':
                self.call(self.target_cli, SelectTarget.Request(track_id=-2), 'target')


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
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
