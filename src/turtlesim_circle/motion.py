"""Convert a revolution period and amplitude into a constant circle command.

The equations use turtlesim's conventional distance units, commonly labelled m.
This module does not import ROS and can be tested on any Python 3.8+ platform.
"""

from dataclasses import dataclass
import math
from typing import Any, Mapping


FLOAT32_MAX = 3.4028234663852886e38


def finite_number(name: str, value: Any, *, positive: bool = False) -> float:
    """Read a finite positive/nonnegative number without accepting booleans."""
    if isinstance(value, bool):
        raise ValueError("{} must be a number, not a boolean".format(name))
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("{} must be a finite number".format(name)) from exc
    if not math.isfinite(number):
        raise ValueError("{} must be finite".format(name))
    if number < 0.0 or (positive and number == 0.0):
        raise ValueError("{} must be {} 0".format(name, ">" if positive else ">="))
    return number


@dataclass(frozen=True)
class CircleParameters:
    period: float = 8.0
    amplitude: float = 1.0
    amplitude_mode: str = "speed"
    direction: int = 1

    @classmethod
    def from_mapping(cls, values: Mapping[str, Any]) -> "CircleParameters":
        period = finite_number("period", values.get("period", 8.0), positive=True)
        amplitude = finite_number("amplitude", values.get("amplitude", 1.0))
        mode = values.get("amplitude_mode", "speed")
        if mode not in ("speed", "radius"):
            raise ValueError("amplitude_mode must be 'speed' or 'radius'")
        raw_direction = values.get("direction", 1)
        if isinstance(raw_direction, bool):
            raise ValueError("direction must be 1 or -1")
        try:
            direction = float(raw_direction)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError("direction must be 1 or -1") from exc
        if direction not in (-1.0, 1.0):
            raise ValueError("direction must be 1 or -1")
        result = cls(period, amplitude, mode, int(direction))
        # Finite inputs may still overflow when divided or multiplied.
        if not all(math.isfinite(value) for value in
                   (result.angular_speed, result.linear_speed, result.radius)):
            raise ValueError("period/amplitude produce non-finite speed or radius")
        if result.angular_speed == 0.0:
            raise ValueError("period is too large to produce a nonzero angular speed")
        # Twist uses float64, but turtlesim publishes velocity in float32 Pose
        # fields. Reject commands that would turn the observed pose into inf.
        if max(abs(result.angular_speed), result.linear_speed) > FLOAT32_MAX:
            raise ValueError("speed exceeds the finite range of turtlesim Pose fields")
        return result

    @property
    def angular_speed(self) -> float:
        return self.direction * 2.0 * math.pi / self.period

    @property
    def linear_speed(self) -> float:
        if self.amplitude_mode == "speed":
            return self.amplitude
        return self.amplitude * abs(self.angular_speed)

    @property
    def radius(self) -> float:
        if self.amplitude_mode == "radius":
            return self.amplitude
        return self.amplitude / abs(self.angular_speed)
