"""Pure gamepad mapping logic (no ROS), unit tested."""
from dataclasses import dataclass, field
from typing import Dict, List, Optional

GAITS = ['tripod', 'ripple', 'wave']


@dataclass
class Command:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    pitch: float = 0.0
    pan_rate: float = 0.0
    tilt_rate: float = 0.0
    actions: List[str] = field(default_factory=list)


def deadband(v: float, band: float = 0.08) -> float:
    if abs(v) < band:
        return 0.0
    return (abs(v) - band) / (1 - band) * (1 if v > 0 else -1)


class Mapper:
    def __init__(self, cfg: Dict[str, float]):
        self.c = cfg
        self.prev: List[int] = []

    def _axis(self, axes, name) -> float:
        i = int(self.c[name])
        return deadband(axes[i]) if 0 <= i < len(axes) else 0.0

    def _button(self, buttons, name) -> bool:
        i = int(self.c[name])
        return 0 <= i < len(buttons) and bool(buttons[i])

    def _pressed(self, buttons, name) -> bool:
        """Rising edge."""
        i = int(self.c[name])
        before = self.prev[i] if i < len(self.prev) else 0
        return self._button(buttons, name) and not before

    def update(self, axes: List[float], buttons: List[int]) -> Command:
        cmd = Command()
        if self._button(buttons, 'button_deadman'):
            k = self.c['fast_factor'] if self._button(buttons, 'button_fast') else 1.0
            # joy convention: stick left / up is positive.
            cmd.vx = self._axis(axes, 'axis_forward') * self.c['max_forward'] * k
            cmd.vy = self._axis(axes, 'axis_sideways') * self.c['max_sideways'] * k
            cmd.wz = self._axis(axes, 'axis_turn') * self.c['max_turn'] * k
            # Stick up raises the nose (negative pitch about +y).
            cmd.pitch = -self._axis(axes, 'axis_pitch') * self.c['max_pitch']
        up = self._button(buttons, 'button_dpad_up')
        down = self._button(buttons, 'button_dpad_down')
        left = self._button(buttons, 'button_dpad_left')
        right = self._button(buttons, 'button_dpad_right')
        cmd.tilt_rate = (up - down) * self.c['head_rate']
        cmd.pan_rate = (left - right) * self.c['head_rate']
        for name, action in (('button_stand', 'stand_sit'), ('button_crouch', 'crouch'),
                             ('button_gait', 'next_gait'), ('button_illuminator', 'illuminator'),
                             ('button_measure', 'measure'), ('button_next_target', 'next_target')):
            if self._pressed(buttons, name):
                cmd.actions.append(action)
        self.prev = list(buttons)
        return cmd


def next_gait(current: Optional[str]) -> str:
    if current not in GAITS:
        return GAITS[0]
    return GAITS[(GAITS.index(current) + 1) % len(GAITS)]
