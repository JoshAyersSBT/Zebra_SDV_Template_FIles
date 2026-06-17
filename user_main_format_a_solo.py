# Zebra SDV: Format A Solo Challenge
# Flash this file as user_main.py for the three-lap solo challenge.
#
# Only external robot helper used here:
# from robot.ackermann import AckermannDrive

import time
import uasyncio as asyncio
from robot.ackermann import AckermannDrive


def show_step(zbot, section, detail="", extra="", more=""):
    """Show the current program section on the display and serial console."""
    zbot.display(section, detail, extra, more)
    print("SDV:", section, detail, extra, more)


# -------------------------
# Student setup values
# -------------------------

# START_GRID must be one of: A2, A3, A4, E2, E3, or E4.
START_GRID = "A3"

# DIRECTION must be "clockwise" or "counterclockwise".
# Place the vehicle facing the first grid square it should drive toward.
DIRECTION = "clockwise"

# Match these ports to the real robot.
DRIVE_MOTOR_PORT = 2
STEERING_PORT = 1

# Tune these angles until the car drives straight and turns cleanly.
CENTER_ANGLE = 90
LEFT_TURN_ANGLE = 55
RIGHT_TURN_ANGLE = 125

# Tune these timing values on the real game mat.
DRIVE_POWER = 35
TURN_POWER = 35
CELL_MS = 1200
TURN_90_MS = 750
SETTLE_MS = 120

# IMU straight-line hold settings.
# Bigger KP means the steering reacts harder when the robot drifts.
# Bigger KD damps fast changes so the car is less likely to wiggle.
# Bigger MAX_CORRECTION_DEG allows stronger steering corrections.
IMU_HOLD_KP = 1.6
IMU_HOLD_KD = 0.18
IMU_HOLD_MAX_CORRECTION_DEG = 30

# IMU turns use the Zebra gyro to stop near 90 degrees instead of guessing
# only by time. If turns count the wrong way, change GYRO_SIGN to -1.
USE_IMU_TURNS = True
GYRO_SIGN = 1
GYRO_DEADBAND_DPS = 0.8
TURN_TIMEOUT_MS = 4500

# The mat has colored grid lines and black square intersections. If a downward
# color sensor is installed, this can count those grid-line crossings.
USE_COLOR_LINES = True
FLOOR_COLOR_PORT = 2
LINE_CLEAR_MAX = 900
LINE_CENTER_MS = 600
LINE_CHECK_MS = 40
LINE_TIMEOUT_FACTOR = 1.5

# Optional safety sensor. Set FRONT_TOF_PORT = None if not installed.
FRONT_TOF_PORT = 1
STOP_DISTANCE_MM = 150


class TimedAckermannCar:
    """Small driving helper kept inside this file so the program is standalone."""

    def __init__(self, zbot):
        self.zbot = zbot
        self.drive = AckermannDrive(
            self.zbot,
            drive_motor_port=DRIVE_MOTOR_PORT,
            steering_port=STEERING_PORT,
            center_angle=CENTER_ANGLE,
            min_angle=45,
            max_angle=135,
            imu_ref=True,
            kp=IMU_HOLD_KP,
            kd=IMU_HOLD_KD,
            max_correction_deg=IMU_HOLD_MAX_CORRECTION_DEG,
            gyro_deadband_dps=GYRO_DEADBAND_DPS,
        )

    def stop(self):
        self.drive.stop()
        self.drive.steer_center()

    async def settle(self):
        self.stop()
        await asyncio.sleep_ms(SETTLE_MS)

    async def forward_cells(self, cells, label="Forward"):
        """Drive forward for a number of grid cells."""
        show_step(self.zbot, "Drive cells", label, "{} cells".format(cells))

        if USE_COLOR_LINES and FLOOR_COLOR_PORT is not None:
            whole_cells = int(cells)
            partial_cells = float(cells) - whole_cells

            for _ in range(whole_cells):
                await self.forward_one_color_cell()

            if partial_cells > 0:
                await self.forward_timed_ms(int(round(partial_cells * CELL_MS)), label)
            return

        await self.forward_timed_ms(int(round(float(cells) * CELL_MS)), label)

    async def forward_timed_ms(self, total_ms, label="Forward"):
        """Drive forward for a fixed number of milliseconds."""
        total_ms = int(total_ms)
        elapsed_ms = 0
        step_ms = 50

        show_step(self.zbot, "Timed drive", label, "{} ms".format(total_ms))
        self.drive.drive(DRIVE_POWER, CENTER_ANGLE)

        try:
            while elapsed_ms < total_ms:
                if self.front_blocked():
                    self.zbot.display("Stopped", "front ToF")
                    break

                # Re-send a straight drive command each loop so the Ackermann
                # helper keeps applying IMU steering corrections.
                self.drive.drive(DRIVE_POWER, CENTER_ANGLE)
                await asyncio.sleep_ms(step_ms)
                elapsed_ms += step_ms
        finally:
            await self.settle()

    async def forward_one_color_cell(self):
        """Drive until the downward color sensor sees the next colored grid line."""
        timeout_ms = int(CELL_MS * LINE_TIMEOUT_FACTOR)
        elapsed_ms = 0
        saw_background = False

        show_step(self.zbot, "Color drive", "looking line")
        self.drive.drive(DRIVE_POWER, CENTER_ANGLE)

        try:
            while elapsed_ms < timeout_ms:
                if self.front_blocked():
                    self.zbot.display("Stopped", "front ToF")
                    break

                # Keep heading hold active while searching for the next line.
                self.drive.drive(DRIVE_POWER, CENTER_ANGLE)

                on_line = self.on_grid_line()
                if not on_line:
                    saw_background = True
                elif saw_background:
                    show_step(self.zbot, "Line found", "centering")
                    await asyncio.sleep_ms(LINE_CENTER_MS)
                    break

                await asyncio.sleep_ms(LINE_CHECK_MS)
                elapsed_ms += LINE_CHECK_MS
        finally:
            await self.settle()

    async def turn_left(self):
        show_step(self.zbot, "Turn command", "left")
        if USE_IMU_TURNS:
            await self.gyro_turn(-90)
        else:
            await self.timed_turn("left")

    async def turn_right(self):
        show_step(self.zbot, "Turn command", "right")
        if USE_IMU_TURNS:
            await self.gyro_turn(90)
        else:
            await self.timed_turn("right")

    async def timed_turn(self, side):
        show_step(self.zbot, "Timed turn", side)
        angle = LEFT_TURN_ANGLE if side == "left" else RIGHT_TURN_ANGLE
        self.drive.drive(TURN_POWER, angle)
        await asyncio.sleep_ms(TURN_90_MS)
        await self.settle()

    async def gyro_turn(self, degrees):
        """Turn until the Zebra IMU gyro reports about 90 degrees of yaw."""
        target = abs(float(degrees))
        direction = 1 if degrees > 0 else -1
        steering = RIGHT_TURN_ANGLE if direction > 0 else LEFT_TURN_ANGLE

        show_step(self.zbot, "Gyro turn", "{} deg".format(int(degrees)), "bias")
        await self.settle()

        bias = await gyro_bias(self.zbot)
        turned = 0.0
        last_ms = ticks_ms()
        start_ms = last_ms

        self.drive.drive(TURN_POWER, steering)

        try:
            while turned < target:
                await asyncio.sleep_ms(10)

                now = ticks_ms()
                dt_s = ticks_diff(now, last_ms) / 1000.0
                last_ms = now

                gz = gyro_z_dps(self.zbot)
                if gz is None:
                    show_step(self.zbot, "Gyro turn", "No IMU", "timed fallback")
                    await self.timed_turn("right" if direction > 0 else "left")
                    return

                rate = (gz - bias) * GYRO_SIGN * direction
                if -GYRO_DEADBAND_DPS < rate < GYRO_DEADBAND_DPS:
                    rate = 0.0

                turned += rate * dt_s
                if turned < 0.0:
                    turned = 0.0

                self.zbot.display("Gyro turn", "{}/{} deg".format(int(turned), int(target)))

                if ticks_diff(now, start_ms) > TURN_TIMEOUT_MS:
                    show_step(self.zbot, "Gyro turn", "timeout")
                    break
        finally:
            await self.settle()

    def on_grid_line(self):
        rgb = self.zbot.rgb(FLOOR_COLOR_PORT)
        if rgb is None:
            return False

        clear = int(rgb.get("clear", 0))
        return clear > 0 and clear <= LINE_CLEAR_MAX

    def front_blocked(self):
        if FRONT_TOF_PORT is None:
            return False

        distance = self.zbot.tof(FRONT_TOF_PORT)
        return distance is not None and distance <= STOP_DISTANCE_MM


def ticks_ms():
    return time.ticks_ms()


def ticks_diff(now, before):
    return time.ticks_diff(now, before)


def imu_payload(zbot):
    """Read the Zebra IMU snapshot from the runtime."""
    api = getattr(zbot, "api", None)
    if api is None:
        return None

    snap = None
    if hasattr(api, "refresh_imu_snapshot"):
        snap = api.refresh_imu_snapshot()
    if snap is None and hasattr(api, "get_imu"):
        snap = api.get_imu()

    if isinstance(snap, dict) and isinstance(snap.get("value"), dict):
        return snap["value"]
    if isinstance(snap, dict):
        return snap
    return None


def gyro_z_dps(zbot):
    """Return gyro Z in degrees per second, or None if no IMU is available."""
    payload = imu_payload(zbot)
    if not isinstance(payload, dict):
        return None

    for key in ("gz_dps", "gyro_z_dps", "gz", "gyro_z"):
        value = payload.get(key)
        if isinstance(value, (int, float)):
            return float(value)
    return None


async def gyro_bias(zbot, samples=25, sample_ms=10):
    """Average the gyro while the robot is still, so turns ignore small drift."""
    total = 0.0
    count = 0

    for _ in range(samples):
        gz = gyro_z_dps(zbot)
        if gz is not None:
            total += gz
            count += 1
        await asyncio.sleep_ms(sample_ms)

    if count == 0:
        return 0.0
    return total / count


def clean_grid(grid):
    """Change 'a3' into ('A', 3) and reject invalid grid names."""
    grid = str(grid).upper().strip()
    if len(grid) != 2:
        raise ValueError("grid must look like A3")

    row = grid[0]
    col = int(grid[1])
    if row not in ("A", "B", "C", "D", "E") or col < 1 or col > 5:
        raise ValueError("grid must be in A1 through E5")

    return row, col


def solo_plan(start, direction):
    """
    Build the outer-loop route.

    The return value is:
    - the corner turn direction: "left" or "right"
    - five straight segments, measured in grid cells
    """
    row, col = clean_grid(start)
    direction = str(direction).lower()

    if direction not in ("clockwise", "counterclockwise"):
        raise ValueError("direction must be clockwise or counterclockwise")

    if row == "A":
        if direction == "clockwise":
            return "right", [5 - col, 4, 4, 4, col - 1]
        return "left", [col - 1, 4, 4, 4, 5 - col]

    if row == "E":
        if direction == "clockwise":
            return "right", [col - 1, 4, 4, 4, 5 - col]
        return "left", [5 - col, 4, 4, 4, col - 1]

    raise ValueError("solo start row must be A or E")


async def run_lap(car, segments, turn):
    # Each lap is straight segment, corner turn, straight segment, and so on.
    for index, cells in enumerate(segments):
        show_step(car.zbot, "Lap segment", str(index + 1), "{} cells".format(cells))
        if cells > 0:
            await car.forward_cells(cells, label="Lap segment")

        if index < 4:
            if turn == "right":
                await car.turn_right()
            else:
                await car.turn_left()


async def main(zbot):
    show_step(zbot, "Format A", "setup", START_GRID, DIRECTION)
    car = TimedAckermannCar(zbot)
    turn, segments = solo_plan(START_GRID, DIRECTION)
    show_step(zbot, "Route plan", "turn {}".format(turn), str(segments))

    try:
        show_step(zbot, "Solo loop", "ready", START_GRID)
        await asyncio.sleep_ms(800)

        # The manual requires three complete laps.
        for lap in range(3):
            show_step(zbot, "Solo lap", str(lap + 1), "start")
            await run_lap(car, segments, turn)
            show_step(zbot, "Solo lap", str(lap + 1), "done")

        show_step(zbot, "Solo loop", "done")

    finally:
        show_step(zbot, "Program", "stopping")
        car.stop()
