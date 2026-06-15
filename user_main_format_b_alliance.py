# Zebra SDV: Format B Alliance Challenge
# Flash this file as user_main.py for the trailer delivery challenge.
#
# Only external robot helper used here:
# from robot.ackermann import AckermannDrive

import time
import uasyncio as asyncio
from robot.ackermann import AckermannDrive


# -------------------------
# Student setup values
# -------------------------

# START_GRID must be one of: A2, A3, A4, E2, E3, or E4.
START_GRID = "A3"

# TRAILER_SIDE chooses which trailer to collect:
# "left" means the C1 trailer, and "right" means the C5 trailer.
TRAILER_SIDE = "left"

# The manual allows up to five trailer placements in one match.
DELIVERIES = 5

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

# Optional hitch servo. Set HITCH_SERVO_PORT = None for a passive hitch.
HITCH_SERVO_PORT = None
HITCH_OPEN_ANGLE = 45
HITCH_CLOSED_ANGLE = 100

# Pause time after the robot asks the judge for the next trailer.
RELOAD_PAUSE_MS = 3000


HEADINGS = ("N", "E", "S", "W")

# A route is a list of grid squares to drive through, one cell at a time.
# The key is (starting square, trailer side).
PICKUP_ROUTES = {
    ("A2", "left"): ["A1", "B1", "C1"],
    ("A3", "left"): ["A2", "A1", "B1", "C1"],
    ("A4", "left"): ["A3", "A2", "A1", "B1", "C1"],
    ("E2", "left"): ["E1", "D1", "C1"],
    ("E3", "left"): ["E2", "E1", "D1", "C1"],
    ("E4", "left"): ["E3", "E2", "E1", "D1", "C1"],
    ("A2", "right"): ["A3", "A4", "A5", "B5", "C5"],
    ("A3", "right"): ["A4", "A5", "B5", "C5"],
    ("A4", "right"): ["A5", "B5", "C5"],
    ("E2", "right"): ["E3", "E4", "E5", "D5", "C5"],
    ("E3", "right"): ["E4", "E5", "D5", "C5"],
    ("E4", "right"): ["E5", "D5", "C5"],
}

# After picking up the trailer, drive from C1 or C5 to D3.
DELIVERY_ROUTES = {
    "left": ["C2", "C3", "D3"],
    "right": ["C4", "C3", "D3"],
}

# After dropping a trailer near D3, drive back to the same pickup square.
RETURN_ROUTES = {
    "left": ["C3", "C2", "C1"],
    "right": ["C3", "C4", "C5"],
}


class TimedAckermannCar:
    """Small driving helper kept inside this file so the program is standalone."""

    def __init__(self, zbot):
        self.zbot = zbot
        self.drive = AckermannDrive(
            zbot,
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
        """Drive forward for a number of grid cells."""
        if USE_COLOR_LINES and FLOOR_COLOR_PORT is not None:
            whole_cells = int(cells)
            partial_cells = float(cells) - whole_cells

            self.zbot.display(label, "{} cells".format(cells))
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

        self.zbot.display(label, "{} ms".format(total_ms))
        self.drive.drive(DRIVE_POWER, CENTER_ANGLE)

        try:
            while elapsed_ms < total_ms:
                if self.front_blocked():
                    self.zbot.display("Stopped", "front ToF")
                    break

                await asyncio.sleep_ms(step_ms)
                elapsed_ms += step_ms
        finally:
            await self.settle()

    async def forward_one_color_cell(self):
        """Drive until the downward color sensor sees the next colored grid line."""
        timeout_ms = int(CELL_MS * LINE_TIMEOUT_FACTOR)
        elapsed_ms = 0
        saw_background = False

        self.drive.drive(DRIVE_POWER, CENTER_ANGLE)

        try:
            while elapsed_ms < timeout_ms:
                if self.front_blocked():
                    self.zbot.display("Stopped", "front ToF")
                    break

                on_line = self.on_grid_line()
                if not on_line:
                    saw_background = True
                elif saw_background:
                    self.zbot.display("Line", "center")
                    await asyncio.sleep_ms(LINE_CENTER_MS)
                    break

                await asyncio.sleep_ms(LINE_CHECK_MS)
                elapsed_ms += LINE_CHECK_MS
        finally:
            await self.settle()

    async def turn_left(self):
        if USE_IMU_TURNS:
            await self.gyro_turn(-90)
        else:
            await self.timed_turn("left")

    async def turn_right(self):
        if USE_IMU_TURNS:
            await self.gyro_turn(90)
        else:
            await self.timed_turn("right")

    async def timed_turn(self, side):
        self.zbot.display("Turn", side)
        angle = LEFT_TURN_ANGLE if side == "left" else RIGHT_TURN_ANGLE
        self.drive.drive(TURN_POWER, angle)
        await asyncio.sleep_ms(TURN_90_MS)
        await self.settle()

    async def turn_around(self):
        await self.turn_right()
        await self.turn_right()

    async def gyro_turn(self, degrees):
        """Turn until the Zebra IMU gyro reports about 90 degrees of yaw."""
        target = abs(float(degrees))
        direction = 1 if degrees > 0 else -1
        steering = RIGHT_TURN_ANGLE if direction > 0 else LEFT_TURN_ANGLE

        self.zbot.display("Gyro turn", "{} deg".format(int(degrees)))
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
                    self.zbot.display("Gyro turn", "No IMU")
                    await self.timed_turn("right" if direction > 0 else "left")
                    return

                rate = (gz - bias) * GYRO_SIGN * direction
                if -GYRO_DEADBAND_DPS < rate < GYRO_DEADBAND_DPS:
                    rate = 0.0

                turned += rate * dt_s
                if turned < 0.0:
                    turned = 0.0

                self.zbot.display("Turning", "{}/{} deg".format(int(turned), int(target)))

                if ticks_diff(now, start_ms) > TURN_TIMEOUT_MS:
                    self.zbot.display("Gyro turn", "timeout")
                    break
        finally:
            await self.settle()

    async def hitch_open(self):
        if HITCH_SERVO_PORT is not None:
            self.zbot.servo(HITCH_SERVO_PORT).angle(HITCH_OPEN_ANGLE)
            await asyncio.sleep_ms(250)

    async def hitch_close(self):
        if HITCH_SERVO_PORT is not None:
            self.zbot.servo(HITCH_SERVO_PORT).angle(HITCH_CLOSED_ANGLE)
            await asyncio.sleep_ms(250)

    def front_blocked(self):
        if FRONT_TOF_PORT is None:
            return False

        distance = self.zbot.tof(FRONT_TOF_PORT)
        return distance is not None and distance <= STOP_DISTANCE_MM

    def on_grid_line(self):
        rgb = self.zbot.rgb(FLOOR_COLOR_PORT)
        if rgb is None:
            return False

        clear = int(rgb.get("clear", 0))
        return clear > 0 and clear <= LINE_CLEAR_MAX


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


def heading_between(src, dst):
    """Return the compass direction needed to drive from one grid cell to the next."""
    row_a, col_a = clean_grid(src)
    row_b, col_b = clean_grid(dst)

    if row_a == row_b and col_b == col_a + 1:
        return "E"
    if row_a == row_b and col_b == col_a - 1:
        return "W"
    if col_a == col_b and ord(row_b) == ord(row_a) + 1:
        return "S"
    if col_a == col_b and ord(row_b) == ord(row_a) - 1:
        return "N"

    raise ValueError("route steps must move to adjacent cells")


def turn_side(current, target):
    """Return how to turn from one heading to another."""
    cur = HEADINGS.index(current)
    nxt = HEADINGS.index(target)
    delta = (nxt - cur) % 4

    if delta == 0:
        return None
    if delta == 1:
        return "right"
    if delta == 3:
        return "left"
    return "around"


async def follow_route(car, current, heading, route):
    """
    Follow a route of grid squares.

    current is the grid square the robot is in now.
    heading is the direction the robot is facing now: N, E, S, or W.
    route is the list of next grid squares to visit.
    """
    if not route:
        return current, heading

    # On the first move, assume the robot was placed facing the first grid square.
    if heading is None:
        heading = heading_between(current, route[0])

    for next_grid in route:
        target_heading = heading_between(current, next_grid)
        turn = turn_side(heading, target_heading)

        if turn == "left":
            await car.turn_left()
        elif turn == "right":
            await car.turn_right()
        elif turn == "around":
            await car.turn_around()

        heading = target_heading
        await car.forward_cells(1, label=next_grid)
        current = next_grid

    return current, heading


async def deliver_one(car, current, heading, trailer, delivery_number):
    """Close the hitch at the pickup cell, drive to D3, then open the hitch."""
    car.zbot.display("Delivery", str(delivery_number), "pickup")
    await car.hitch_close()
    await asyncio.sleep_ms(250)

    current, heading = await follow_route(
        car,
        current,
        heading,
        DELIVERY_ROUTES[trailer],
    )

    car.zbot.display("Delivery", str(delivery_number), "drop")
    await car.hitch_open()
    await asyncio.sleep_ms(400)
    return current, heading


async def main(zbot):
    car = TimedAckermannCar(zbot)
    start = START_GRID.upper()
    trailer = TRAILER_SIDE.lower()
    deliveries = int(DELIVERIES)
    current = start
    heading = None

    try:
        zbot.display("Alliance", "{} {}".format(start, trailer))
        await car.hitch_open()
        await asyncio.sleep_ms(800)

        current, heading = await follow_route(
            car,
            current,
            heading,
            PICKUP_ROUTES[(start, trailer)],
        )
        current, heading = await deliver_one(car, current, heading, trailer, 1)

        for delivery in range(2, deliveries + 1):
            zbot.display("Request", "trailer {}".format(delivery))
            await asyncio.sleep_ms(RELOAD_PAUSE_MS)

            current, heading = await follow_route(
                car,
                current,
                heading,
                RETURN_ROUTES[trailer],
            )
            current, heading = await deliver_one(car, current, heading, trailer, delivery)

        zbot.display("Alliance", "done")

    finally:
        car.stop()
