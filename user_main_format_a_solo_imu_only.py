# Zebra SDV: Format A Solo Challenge, IMU Only
# Flash this file as user_main.py for the three-lap solo challenge.
#
# This version uses:
# - AckermannDrive for the drive motor and steering servo
# - the Zebra IMU gyro for turns
# - timing for straight grid-cell driving
#
# It does not use color sensors or time-of-flight sensors.

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

# Tune CELL_MS until one grid-cell move is correct on the mat.
DRIVE_POWER = 35
CELL_MS = 1200
SETTLE_MS = 120

# IMU turn settings. If turns count the wrong way, change GYRO_SIGN to -1.
TURN_POWER = 35
GYRO_SIGN = 1
GYRO_DEADBAND_DPS = 0.8
TURN_TIMEOUT_MS = 4500

# If no IMU reading is found, the program falls back to this timed turn.
TURN_90_MS = 750


class ImuOnlyAckermannCar:
    """Driving helper for timed straight moves and IMU-measured turns."""

    def __init__(self, zbot):
        self.zbot = zbot
        self.drive = AckermannDrive(
            self.zbot,
            drive_motor_port=DRIVE_MOTOR_PORT,
            steering_port=STEERING_PORT,
            center_angle=CENTER_ANGLE,
            min_angle=45,
            max_angle=135,
        )

    def stop(self):
        self.drive.stop()
        self.drive.steer_center()

    async def settle(self):
        self.stop()
        await asyncio.sleep_ms(SETTLE_MS)

    async def forward_cells(self, cells, label="Forward"):
        """Drive straight for a timed number of grid cells."""
        total_ms = int(round(float(cells) * CELL_MS))
        elapsed_ms = 0
        step_ms = 50

        show_step(self.zbot, "Timed drive", label, "{} cells".format(cells))
        self.drive.drive(DRIVE_POWER, CENTER_ANGLE)

        try:
            while elapsed_ms < total_ms:
                await asyncio.sleep_ms(step_ms)
                elapsed_ms += step_ms
        finally:
            await self.settle()

    async def turn_left(self):
        show_step(self.zbot, "Turn command", "left")
        await self.gyro_turn(-90)

    async def turn_right(self):
        show_step(self.zbot, "Turn command", "right")
        await self.gyro_turn(90)

    async def timed_turn(self, side):
        """Backup turn used only when the IMU is not available."""
        angle = LEFT_TURN_ANGLE if side == "left" else RIGHT_TURN_ANGLE
        show_step(self.zbot, "Timed turn", side)
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
    show_step(zbot, "Format A IMU", "setup", START_GRID, DIRECTION)
    car = ImuOnlyAckermannCar(zbot)
    turn, segments = solo_plan(START_GRID, DIRECTION)
    show_step(zbot, "Route plan", "turn {}".format(turn), str(segments))

    try:
        show_step(zbot, "Solo IMU", "ready", START_GRID)
        await asyncio.sleep_ms(800)

        for lap in range(3):
            show_step(zbot, "Solo lap", str(lap + 1), "start")
            await run_lap(car, segments, turn)
            show_step(zbot, "Solo lap", str(lap + 1), "done")

        show_step(zbot, "Solo IMU", "done")

    finally:
        show_step(zbot, "Program", "stopping")
        car.stop()
