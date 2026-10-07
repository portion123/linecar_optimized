"""Independent physical plant for testing eight-probe square-track firmware.

This module deliberately contains no line-following controller. Tests supply the
actual control C source in a native host harness's signed PWM, and consume this model's sensor bit mask
and encoder pulses. Motor/encoder hardware polarity belongs in the adapter.
Wheel diameter 44 mm, axle track 133 mm and front offset 175 mm were confirmed
by the user on 2026-10-01. Probe pitch 12 mm, line width 20 mm and the motor
response/friction remain model assumptions and must be checked on the car.
"""

from dataclasses import dataclass
from math import atan2, cos, exp, hypot, pi, sin


@dataclass
class SquareConfig:
    side_m: float = 1.0
    line_width_m: float = 0.020
    wheel_diameter_m: float = 0.044
    axle_track_m: float = 0.133
    front_offset_m: float = 0.175
    probe_pitch_m: float = 0.012
    # 用户的两次10圈计数，以及30/40%空转PPS；控制器不会读取这个模型。
    left_counts_per_rev: float = 251.0
    right_counts_per_rev: float = 265.35
    left_gain_rpm_per_pwm: float = 20.43824701
    right_gain_rpm_per_pwm: float = 20.03391747
    left_dead_pwm: float = 6.47887324
    right_dead_pwm: float = 4.67268623
    # 起转门槛、响应时间和低速外推仍为假设，不是30/40%两点能测出来的量。
    left_breakaway_pwm: float = 14.0
    right_breakaway_pwm: float = 16.0
    left_response_s: float = 0.12
    right_response_s: float = 0.20


def point_segment_distance(x, y, ax, ay, bx, by):
    """Return distance and bounded projection parameter to a finite segment."""
    dx, dy = bx - ax, by - ay
    denom = dx * dx + dy * dy
    t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / denom))
    return hypot(x - (ax + t * dx), y - (ay + t * dy)), t


class SquarePlant:
    """Differential drive with unequal wheel loads and a FRONT sensor array.

    ``direction=+1`` follows the square counterclockwise (four left turns).
    ``direction=-1`` follows it clockwise (four right turns). Both starts travel
    toward world +x, so probe 1 is always on the physical left of the vehicle.
    ``lateral_offset_m`` is signed in world y; ``heading_error_rad`` is signed
    left yaw. All geometry is in metres, time seconds, wheel speed RPM.

    Interface:
      mask = plant.sensor_mask()       # bit 0 = physical LEFT probe
      counts = plant.advance(l, r, dt) # signed physical-forward encoder pulses
      plant.travel_m                  # accumulated progress in intended direction
      plant.completed_corners         # geometric corner boundaries passed
      plant.nearest_track()           # distance, projection around perimeter, edge

    For the currently known firmware, adapt physical encoder pulses to raw
    counters as ``(-counts[0], counts[1])``. Read the actual configured signs if
    they change. Feed ``~mask & 0xff`` into GPIO only for active-low inputs.
    """

    def __init__(self, direction=1, lateral_offset_m=0.0, heading_error_rad=0.0,
                 config=None, start_distance_m=0.12):
        if direction not in (-1, 1):
            raise ValueError('direction must be +1 or -1')
        self.config = config or SquareConfig()
        if self.config.front_offset_m <= 0 or self.config.axle_track_m <= 0:
            raise ValueError('front offset and axle track must be positive')
        self.direction = direction
        self.x = start_distance_m
        self.y = lateral_offset_m + (0.0 if direction == 1 else self.config.side_m)
        self.heading = heading_error_rad
        self.rpm = [0.0, 0.0]
        self.encoder_residual = [0.0, 0.0]
        self.time_s = 0.0
        self.travel_m = 0.0
        self.max_center_distance_m = 0.0
        self.zero_sensor_time_s = 0.0
        self.longest_zero_sensor_s = 0.0
        self.initial_progress_m = self.nearest_track()[1]
        self.previous_progress_m = self.initial_progress_m
        self.initial_heading = self.heading
        self.pose_history = [(self.time_s, self.x, self.y, self.heading)]

    @property
    def perimeter_m(self):
        return self.config.side_m * 4.0

    @property
    def completed_corners(self):
        side = self.config.side_m
        start = self.initial_progress_m
        end = start + self.direction * self.travel_m
        if self.direction == 1:
            return max(0, int(end // side) - int(start // side))
        return max(0, int(start // side) - int(end // side))

    @property
    def laps(self):
        return self.travel_m / self.perimeter_m

    @property
    def yaw_turns(self):
        return self.direction * (self.heading - self.initial_heading) / (2.0 * pi)

    def nearest_track(self, x=None, y=None):
        x = self.x if x is None else x
        y = self.y if y is None else y
        s = self.config.side_m
        segments = ((0., 0., s, 0.), (s, 0., s, s),
                    (s, s, 0., s), (0., s, 0., 0.))
        distances = [point_segment_distance(x, y, *segment) for segment in segments]
        edge = min(range(4), key=lambda i: distances[i][0])
        distance, along = distances[edge]
        return distance, (edge + along) * s, edge

    def sensor_points(self):
        d = self.config.front_offset_m
        c, s = cos(self.heading), sin(self.heading)
        # Probe index 0 is physical LEFT (+local y); index 7 is RIGHT.
        lateral = [(3.5 - i) * self.config.probe_pitch_m for i in range(8)]
        return [(self.x + d * c - side * s, self.y + d * s + side * c)
                for side in lateral]

    def sensor_mask(self):
        half_width = self.config.line_width_m * 0.5
        return sum(1 << i for i, (x, y) in enumerate(self.sensor_points())
                   if self.nearest_track(x, y)[0] <= half_width + 1e-12)

    def _update_wheels(self, left_pwm, right_pwm, dt):
        cfg = self.config
        parameters = ((cfg.left_gain_rpm_per_pwm, cfg.left_dead_pwm,
                       cfg.left_breakaway_pwm, cfg.left_response_s),
                      (cfg.right_gain_rpm_per_pwm, cfg.right_dead_pwm,
                       cfg.right_breakaway_pwm, cfg.right_response_s))
        for i, (pwm, param) in enumerate(zip((left_pwm, right_pwm), parameters)):
            gain, dead, breakaway, tau = param
            magnitude = min(100.0, abs(float(pwm)))
            sign = -1.0 if pwm < 0 else 1.0
            if abs(self.rpm[i]) < 0.8 and magnitude < breakaway:
                equilibrium = 0.0
            else:
                equilibrium = sign * gain * max(0.0, magnitude - dead)
            self.rpm[i] += (1.0 - exp(-dt / tau)) * (equilibrium - self.rpm[i])

    def advance(self, left_pwm, right_pwm, dt=0.02):
        if not 0.0 < dt <= 0.2:
            raise ValueError('dt must be positive and at most 200 ms')
        previous_rpm = self.rpm[:]
        self._update_wheels(left_pwm, right_pwm, dt)
        # Trapezoidal wheel integration permits realistic acceleration and coast.
        integrated_rpm = [(old + new) * 0.5 for old, new in zip(previous_rpm, self.rpm)]
        circumference = pi * self.config.wheel_diameter_m
        left_v, right_v = [rpm * circumference / 60.0 for rpm in integrated_rpm]
        linear = (left_v + right_v) * 0.5
        angular = (right_v - left_v) / self.config.axle_track_m
        delta_heading = angular * dt
        if abs(angular) < 1e-10:
            self.x += linear * dt * cos(self.heading)
            self.y += linear * dt * sin(self.heading)
        else:
            radius = linear / angular
            self.x += radius * (sin(self.heading + delta_heading) - sin(self.heading))
            self.y -= radius * (cos(self.heading + delta_heading) - cos(self.heading))
        self.heading += delta_heading
        self.time_s += dt
        counts = []
        for i, rpm in enumerate(integrated_rpm):
            counts_per_rev = (self.config.left_counts_per_rev, self.config.right_counts_per_rev)[i]
            self.encoder_residual[i] += rpm * counts_per_rev * dt / 60.0
            count = int(self.encoder_residual[i])  # signed truncation, residual retained
            self.encoder_residual[i] -= count
            counts.append(count)
        distance, progress, _ = self.nearest_track()
        delta = (progress - self.previous_progress_m + self.perimeter_m / 2.0) % self.perimeter_m
        delta -= self.perimeter_m / 2.0
        self.travel_m += self.direction * delta
        self.previous_progress_m = progress
        self.max_center_distance_m = max(self.max_center_distance_m, distance)
        if self.sensor_mask() == 0:
            self.zero_sensor_time_s += dt
            self.longest_zero_sensor_s = max(self.longest_zero_sensor_s, self.zero_sensor_time_s)
        else:
            self.zero_sensor_time_s = 0.0
        self.pose_history.append((self.time_s, self.x, self.y, self.heading))
        return tuple(counts)


def self_check():
    """Sanity checks of geometry and physical direction, independent of control."""
    centered = SquarePlant(config=SquareConfig(line_width_m=0.014))
    assert centered.sensor_mask() == 0x18
    left_line = SquarePlant(lateral_offset_m=-0.03,
                            config=SquareConfig(line_width_m=0.014))
    right_line = SquarePlant(lateral_offset_m=0.03,
                             config=SquareConfig(line_width_m=0.014))
    assert left_line.sensor_mask() & 0x03
    assert right_line.sensor_mask() & 0xC0
    symmetric = SquareConfig(left_gain_rpm_per_pwm=20.0,right_gain_rpm_per_pwm=20.0,
                             left_dead_pwm=6.0,right_dead_pwm=6.0,
                             left_breakaway_pwm=14.0,right_breakaway_pwm=14.0,
                             left_response_s=0.12,right_response_s=0.12)
    plant = SquarePlant(config=symmetric)
    for _ in range(100):
        plant.advance(-40, 40)
    assert plant.heading > 0 and abs(plant.x - 0.12) < 1e-9 and abs(plant.y) < 1e-9
    plant = SquarePlant(config=symmetric)
    for _ in range(100):
        plant.advance(40, -40)
    assert plant.heading < 0
    bench = SquarePlant()
    for _ in range(100):
        bench.advance(40, 40)
    assert bench.rpm[1] > bench.rpm[0]  # 用户40%实测右轮真实RPM略大
    # At a front-sensor corner detection, an immediate axle pivot misses the
    # outgoing strip. The controller must advance the axle before that pivot.
    plant = SquarePlant(config=SquareConfig(line_width_m=0.014))
    plant.x = plant.config.side_m - plant.config.front_offset_m
    plant.heading = pi / 2
    assert plant.sensor_mask() == 0
    plant.x = plant.config.side_m
    assert plant.sensor_mask() == 0x18
    print('PASS: front-array square geometry, physical turn polarity, unequal load, corner axle placement')


if __name__ == '__main__':
    self_check()
