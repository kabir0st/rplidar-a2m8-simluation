"""3-wheel differential-drive robot with the lidar mounted on top.

The motor command interface mirrors the SPI control board defined in
spi_com.h (TYPE_White_Board_TX/RX): the host sends signed-short Set_Speed
or Set_PWM values per motor, and the board reports Speed and accumulated
encoder Position. The simulation accepts the same numbers a SLAM controller
would send to the real hardware so behaviour matches across sim and robot.

Geometry: 2 driven wheels on a rear axle, 1 passive caster at the front,
lidar mounted at chassis center (forward of the rear axle).

All pose math is done in canvas pixels so the rest of the simulator (ray
casting, UI rendering) keeps using the same units it always has.
"""

import math
import threading
import time

# Maxon DCX 22 S 12V + GPX 22 6.6:1 + ENX 10 EASY 1024 CPT (see maxon_12v.pdf)
WHEEL_DIAMETER_MM = 80
WHEELBASE_MM = 200          # left-to-right driven wheel separation
CHASSIS_LENGTH_MM = 280     # rear axle to front caster
CHASSIS_WIDTH_MM = 240      # body width (slightly wider than wheelbase)
LIDAR_OFFSET_MM = 80        # lidar sits this far forward of the rear axle
GEAR_RATIO = 6.6
ENCODER_CPT = 1024          # counts per turn at the motor shaft
CONTROLLER_TICK_HZ = 100    # SPI controller tick (matches usleep(10000) in spitest.cpp)

# Motor max: 6200 rpm no-load / 6.6 gearbox = ~939 rpm wheel = ~98.4 rad/s.
NO_LOAD_MOTOR_RPM = 6200
MAX_WHEEL_OMEGA = (NO_LOAD_MOTOR_RPM / GEAR_RATIO) * 2.0 * math.pi / 60.0

# Set_Speed is in encoder-counts per controller tick. Convert to wheel rad/s:
#   counts/tick * tick_hz / counts_per_wheel_rev * 2*pi
SET_SPEED_TO_WHEEL_OMEGA = (
    CONTROLLER_TICK_HZ * 2.0 * math.pi / (ENCODER_CPT * GEAR_RATIO)
)
# And the inverse, used when accumulating position from saturated wheel speed.
WHEEL_OMEGA_TO_SET_SPEED = 1.0 / SET_SPEED_TO_WHEEL_OMEGA

# PWM mapping: Set_PWM in [-PWM_FULL_SCALE, +PWM_FULL_SCALE] -> [-12V, +12V].
PWM_FULL_SCALE = 1000
NOMINAL_VOLTAGE = 12.0
SPEED_CONSTANT_RPM_PER_V = 520.0    # from datasheet

INT16_MIN, INT16_MAX = -32768, 32767

TICKER_HZ = 100
TICKER_PERIOD = 1.0 / TICKER_HZ


def _clamp_int16(v):
    return max(INT16_MIN, min(INT16_MAX, int(v)))


class Robot:
    def __init__(self, width_px, height_px, mm_per_pixel):
        self.width_px = width_px
        self.height_px = height_px
        self.mm_per_pixel = mm_per_pixel

        self.wheel_radius_m = (WHEEL_DIAMETER_MM / 2.0) / 1000.0
        self.wheelbase_m = WHEELBASE_MM / 1000.0

        # Pose: rear-axle midpoint, in canvas pixels. theta=0 points +x (east).
        self.x_px = width_px / 2.0
        self.y_px = height_px / 2.0
        self.theta_rad = 0.0

        # TX state (commanded; mirrors TYPE_White_Board_TX from spi_com.h).
        self.mode = "speed"            # "speed" or "pwm"
        self.set_speed_m0 = 0          # left
        self.set_speed_m1 = 0          # right
        self.set_pwm_m0 = 0
        self.set_pwm_m1 = 0

        # RX state (measured; mirrors TYPE_White_Board_RX from spi_com.h).
        self.speed_m0 = 0              # signed short, same units as Set_Speed
        self.speed_m1 = 0
        self.position_m0 = 0           # encoder counts, integrated
        self.position_m1 = 0

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = None

    # ----- public motor API (mirrors spi_com.h) ------------------------------

    def set_motor_speeds(self, left, right):
        """Closed-loop speed setpoint per wheel (Set_Speed_M0/M1 semantics)."""
        with self._lock:
            self.mode = "speed"
            self.set_speed_m0 = _clamp_int16(left)
            self.set_speed_m1 = _clamp_int16(right)

    def set_motor_pwm(self, left, right):
        """Open-loop PWM duty per wheel (Set_PWM_M0/M1 semantics)."""
        with self._lock:
            self.mode = "pwm"
            self.set_pwm_m0 = _clamp_int16(left)
            self.set_pwm_m1 = _clamp_int16(right)

    def stop(self):
        with self._lock:
            self.set_speed_m0 = 0
            self.set_speed_m1 = 0
            self.set_pwm_m0 = 0
            self.set_pwm_m1 = 0

    # ----- pose access -------------------------------------------------------

    def set_pose(self, x_px=None, y_px=None, theta_rad=None):
        with self._lock:
            if x_px is not None:
                self.x_px = max(1.0, min(self.width_px - 1.0, float(x_px)))
            if y_px is not None:
                self.y_px = max(1.0, min(self.height_px - 1.0, float(y_px)))
            if theta_rad is not None:
                self.theta_rad = float(theta_rad) % (2.0 * math.pi)

    def get_lidar_pose(self):
        """Return (lidar_x_px, lidar_y_px, heading_deg) for ray casting."""
        with self._lock:
            offset_px = (LIDAR_OFFSET_MM / self.mm_per_pixel)
            lx = self.x_px + offset_px * math.cos(self.theta_rad)
            ly = self.y_px + offset_px * math.sin(self.theta_rad)
            heading_deg = math.degrees(self.theta_rad) % 360.0
        return lx, ly, heading_deg

    def get_chassis_snapshot(self):
        """Pose + motor command/measurement values for UI rendering."""
        with self._lock:
            return {
                "x_px": self.x_px,
                "y_px": self.y_px,
                "theta_rad": self.theta_rad,
                "mode": self.mode,
                "set_speed_m0": self.set_speed_m0,
                "set_speed_m1": self.set_speed_m1,
                "set_pwm_m0": self.set_pwm_m0,
                "set_pwm_m1": self.set_pwm_m1,
                "speed_m0": self.speed_m0,
                "speed_m1": self.speed_m1,
                "position_m0": self.position_m0,
                "position_m1": self.position_m1,
            }

    # ----- kinematics ticker -------------------------------------------------

    def start_ticker(self):
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._ticker_loop, daemon=True)
        self._thread.start()

    def stop_ticker(self):
        self._stop_event.set()

    def _ticker_loop(self):
        last = time.monotonic()
        while not self._stop_event.is_set():
            now = time.monotonic()
            dt = now - last
            last = now
            # Cap dt so a stalled scheduler can't teleport the robot.
            if dt > 0.1:
                dt = 0.1
            self._step(dt)
            sleep_for = TICKER_PERIOD - (time.monotonic() - now)
            if sleep_for > 0:
                time.sleep(sleep_for)

    def _commanded_wheel_omegas(self):
        """Return (omega_left, omega_right) wheel angular velocities (rad/s)."""
        if self.mode == "pwm":
            v_l = (self.set_pwm_m0 / PWM_FULL_SCALE) * NOMINAL_VOLTAGE
            v_r = (self.set_pwm_m1 / PWM_FULL_SCALE) * NOMINAL_VOLTAGE
            motor_rpm_l = v_l * SPEED_CONSTANT_RPM_PER_V
            motor_rpm_r = v_r * SPEED_CONSTANT_RPM_PER_V
            wheel_omega_l = (motor_rpm_l / GEAR_RATIO) * 2.0 * math.pi / 60.0
            wheel_omega_r = (motor_rpm_r / GEAR_RATIO) * 2.0 * math.pi / 60.0
        else:
            wheel_omega_l = self.set_speed_m0 * SET_SPEED_TO_WHEEL_OMEGA
            wheel_omega_r = self.set_speed_m1 * SET_SPEED_TO_WHEEL_OMEGA
        wheel_omega_l = max(-MAX_WHEEL_OMEGA, min(MAX_WHEEL_OMEGA, wheel_omega_l))
        wheel_omega_r = max(-MAX_WHEEL_OMEGA, min(MAX_WHEEL_OMEGA, wheel_omega_r))
        return wheel_omega_l, wheel_omega_r

    def _step(self, dt):
        with self._lock:
            omega_l, omega_r = self._commanded_wheel_omegas()

            v_l_m = omega_l * self.wheel_radius_m
            v_r_m = omega_r * self.wheel_radius_m
            v_m = (v_l_m + v_r_m) / 2.0
            w = (v_r_m - v_l_m) / self.wheelbase_m

            # m/s -> px/s
            v_px = v_m * 1000.0 / self.mm_per_pixel

            self.x_px += v_px * math.cos(self.theta_rad) * dt
            self.y_px += v_px * math.sin(self.theta_rad) * dt
            self.theta_rad = (self.theta_rad + w * dt) % (2.0 * math.pi)

            # Keep robot inside the arena (chassis center can't leave it).
            self.x_px = max(1.0, min(self.width_px - 1.0, self.x_px))
            self.y_px = max(1.0, min(self.height_px - 1.0, self.y_px))

            # RX measurements: ideal model — measured speed = commanded
            # equivalent (after saturation). Position integrates from speed.
            ss_l = _clamp_int16(round(omega_l * WHEEL_OMEGA_TO_SET_SPEED))
            ss_r = _clamp_int16(round(omega_r * WHEEL_OMEGA_TO_SET_SPEED))
            self.speed_m0 = ss_l
            self.speed_m1 = ss_r
            # counts/sec at the motor shaft = wheel_omega * gear * cpt / (2*pi)
            counts_per_sec_l = omega_l * GEAR_RATIO * ENCODER_CPT / (2.0 * math.pi)
            counts_per_sec_r = omega_r * GEAR_RATIO * ENCODER_CPT / (2.0 * math.pi)
            self.position_m0 += int(round(counts_per_sec_l * dt))
            self.position_m1 += int(round(counts_per_sec_r * dt))
