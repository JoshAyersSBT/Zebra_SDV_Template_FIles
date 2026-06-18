# Zebra SDV: Format A Grand Loop, Color Line Version
# Flash this file as user_main.py for the Format A solo challenge.
#
# This file was forked from the Ackermann square test. It keeps the same:
# - AckermannDrive helper
# - IMU gyro turns
# - Zebra color sensor line detection for grid-square movement
#
# Format A goal from the game manual:
# - Start in A2, A3, A4, E2, E3, or E4.
# - Drive around the outside loop of the mat.
# - Avoid the restricted center squares B2-B4, C2-C4, and D2-D4.
# - Complete three full laps.
# - Stop back inside the original starting square.
#
# Placement assumption:
# Put the robot inside START_GRID, facing the next grid square in the chosen
# direction. The colored grid lines are not part of the starting area.
import time
import uasyncio as asyncio
from robot.ackermann import AckermannDrive



async def main(zbot):
    # -------------------------
    # Challenge choices
    # -------------------------

    # START_GRID must be one of: A2, A3, A4, E2, E3, or E4.
    START_GRID = "A4"

    # DIRECTION must be "clockwise" or "counterclockwise".
    DIRECTION = "clockwise"

    # Format A requires three laps.
    LAPS_TO_RUN = 3

    # -------------------------
    # Robot setup values
    # -------------------------

    DRIVE_MOTOR_PORT = 2
    STEERING_PORT = 1

    # Use the servo center finder result here.
    CENTER_ANGLE = 90

    # RIGHT_TURN_ANGLE and LEFT_TURN_ANGLE are near the steering limits so the
    # robot can make tighter Ackermann turns.
    RIGHT_TURN_ANGLE = 132
    LEFT_TURN_ANGLE = 48

    # Use the signed drive power from the servo center finder if needed.
    DRIVE_POWER = 35
    TURN_POWER = 30

    # Downward Zebra color sensor used to find the colored grid lines.
    FLOOR_COLOR_PORT = 2

    # Clear-channel threshold for detecting a colored or dark grid line.
    # Use user_main_sensor_check.py to tune this. The value over a line should
    # be lower than the value over the white mat background.
    LINE_CLEAR_MAX = 900

    # If the Zebra color API can name the line color, use that as a sanity
    # check. For example, green should not appear right after brown on the top
    # row; after brown, the next clockwise line should be magenta.
    USE_COLOR_NAME_SANITY = True
    COLOR_CONFIDENCE_MIN = 20

    # The API can sometimes call the mint/cyan/aqua line "green". Use raw RGB
    # balance to split those apart: true green has much less blue than green,
    # while cyan/mint/aqua has blue close to green.
    CYAN_BLUE_TO_GREEN_MIN = 0.55
    GREEN_BLUE_TO_GREEN_MAX = 0.40
    GREEN_RED_TO_GREEN_MAX = 0.75

    # After the sensor sees a line, keep driving briefly so the robot's body
    # moves off the line and into the next grid square.
    LINE_CENTER_MS = 250

    # Corners need a shorter coast after the line so the turn starts earlier
    # and the robot does not overshoot deep into the corner square.
    CORNER_LINE_CENTER_MS = 120

    # How often to check the color sensor while driving.
    LINE_CHECK_MS = 40

    # Require the expected line to be seen for several checks before accepting
    # the crossing. This rejects single-sample color glitches.
    LINE_MATCH_CONFIRMATIONS = 3

    # Safety stop for one grid-square movement. The line sensor is still the
    # main location signal, but this prevents driving forever if the sensor is
    # unplugged or the threshold is wrong.
    LINE_TIMEOUT_MS = 4000

    # The robot was underturning by about 5 degrees, so 95 is used to make the
    # real-world turn closer to 90 degrees.
    TURN_90_DEG = 95

    # Positioning delay before the run starts.
    START_DELAY_S = 10

    # Loop timing.
    LOOP_MS = 20

    # Small gyro readings are usually sensor noise.
    GYRO_DEADBAND_DPS = 1.0

    # Moderate straight-line heading hold to avoid steering shake.
    IMU_HOLD_KP = 1.1
    IMU_HOLD_KD = 0.08
    IMU_HOLD_MAX_CORRECTION_DEG = 18

    # The legal outer loop squares, written clockwise around the mat.
    OUTER_LOOP = (
        "A1", "A2", "A3", "A4", "A5",
        "B5", "C5", "D5", "E5",
        "E4", "E3", "E2", "E1",
        "D1", "C1", "B1",
    )

    CORNER_GRIDS = ("A1", "A5", "E5", "E1")

    # Color map from SDV_GM_v3.pdf, Fig. 5 "Grand loop Ride".
    # These are the expected colored grid lines between adjacent squares.
    COLUMN_CROSSING_COLORS = {
        (1, 2): "green",
        (2, 3): "cyan",
        (3, 4): "brown",
        (4, 5): "magenta",
    }

    ROW_CROSSING_COLORS = {
        ("A", "B"): "yellow",
        ("B", "C"): "blue",
        ("C", "D"): "red",
        ("D", "E"): "purple",
    }

    COLOR_ALIASES = {
        "aqua": "cyan",
        "teal": "cyan",
        "turquoise": "cyan",
        "sky_blue": "cyan",
        "light_blue": "cyan",
        "pink": "magenta",
        "fuchsia": "magenta",
        "violet": "purple",
    }

    car = AckermannDrive(
        zbot,
        drive_motor_port=DRIVE_MOTOR_PORT,
        steering_port=STEERING_PORT,
        center_angle=CENTER_ANGLE,
        imu_ref=True,
        kp=IMU_HOLD_KP,
        kd=IMU_HOLD_KD,
        max_correction_deg=IMU_HOLD_MAX_CORRECTION_DEG,
        gyro_deadband_dps=GYRO_DEADBAND_DPS,
    )

    def show_step(line1, line2="", line3=""):
        """Show the current code section on the display and serial console."""
        zbot.display(line1, line2, line3)
        print("FORMAT_A:", line1, line2, line3)

    def gyro_z_dps():
        """Read the Zebra IMU Z gyro in degrees per second."""
        imu = zbot.imu()
        value = imu.get("value", {}) if imu else {}
        gz = value.get("gz_dps")

        if gz is None:
            return 0.0

        gz = float(gz)
        if -GYRO_DEADBAND_DPS < gz < GYRO_DEADBAND_DPS:
            return 0.0

        return gz

    async def stop_and_center():
        """Stop the drive motor and point the steering straight ahead."""
        car.stop()
        car.steer_center()
        await asyncio.sleep_ms(250)

    async def wait_for_positioning():
        """Give the team time to place the robot before autonomous motion."""
        for seconds_left in range(START_DELAY_S, 0, -1):
            show_step("Position robot", START_GRID, "{} sec".format(seconds_left))
            await asyncio.sleep_ms(1000)

        show_step("Starting", START_GRID, DIRECTION)
        await asyncio.sleep_ms(300)

    def next_square(square):
        """Return the next outer-loop square for the chosen direction."""
        index = OUTER_LOOP.index(square)
        if DIRECTION == "clockwise":
            return OUTER_LOOP[(index + 1) % len(OUTER_LOOP)]
        return OUTER_LOOP[(index - 1) % len(OUTER_LOOP)]

    def turn_side_for_direction():
        """Clockwise uses right turns; counterclockwise uses left turns."""
        if DIRECTION == "clockwise":
            return "right"
        return "left"

    def expected_line_color(from_grid, to_grid):
        """Return the expected color between two adjacent grid squares."""
        from_row = from_grid[0]
        to_row = to_grid[0]
        from_col = int(from_grid[1])
        to_col = int(to_grid[1])

        if from_row == to_row:
            low = min(from_col, to_col)
            high = max(from_col, to_col)
            return COLUMN_CROSSING_COLORS.get((low, high), "unknown")

        if from_col == to_col:
            pair = tuple(sorted((from_row, to_row)))
            return ROW_CROSSING_COLORS.get(pair, "unknown")

        return "unknown"

    def floor_clear_value():
        """Read the color sensor clear channel from the Zebra color API."""
        rgb = zbot.rgb(FLOOR_COLOR_PORT)
        if rgb is None:
            return None

        return int(rgb.get("clear", 0))

    def floor_rgb():
        """Read raw RGB from the Zebra color sensor."""
        return zbot.rgb(FLOOR_COLOR_PORT)

    def normalize_color_name(color):
        """Normalize color names from the Zebra color API."""
        if color is None:
            return None

        name = str(color).strip().lower().replace("-", "_").replace(" ", "_")
        return COLOR_ALIASES.get(name, name)

    def rgb_color_hint(rgb):
        """Use raw RGB balance to avoid green/cyan mixups."""
        if not isinstance(rgb, dict):
            return None

        r = int(rgb.get("r", 0))
        g = int(rgb.get("g", 0))
        b = int(rgb.get("b", 0))

        if g <= 0:
            return None

        blue_to_green = b / float(g)
        red_to_green = r / float(g)

        if blue_to_green >= CYAN_BLUE_TO_GREEN_MIN:
            return "cyan"

        if (
            blue_to_green <= GREEN_BLUE_TO_GREEN_MAX
            and red_to_green <= GREEN_RED_TO_GREEN_MAX
        ):
            return "green"

        return None

    def detected_line_color(rgb):
        """Return the best line color estimate, preferring raw RGB sanity."""
        rgb_hint = rgb_color_hint(rgb)
        if rgb_hint is not None:
            return rgb_hint

        match = zbot.color_match(FLOOR_COLOR_PORT)
        if isinstance(match, dict):
            confidence = int(match.get("confidence", 0))
            color = normalize_color_name(match.get("color"))
            if color is not None and confidence >= COLOR_CONFIDENCE_MIN:
                return color

        return normalize_color_name(zbot.color(FLOOR_COLOR_PORT))

    def on_grid_line(clear):
        """Return True when the floor sensor sees a colored grid line."""
        if clear is None:
            return False

        return clear > 0 and clear <= LINE_CLEAR_MAX

    def line_color_is_sane(detected_color, expected_color):
        """Return True when the detected color is plausible for this route step."""
        if not USE_COLOR_NAME_SANITY:
            return True

        # Some sensors expose only clear/RGB values and no useful color name.
        # In that case, keep using clear-threshold line detection.
        if detected_color is None:
            return True

        if expected_color == "unknown":
            return True

        return detected_color == expected_color

    async def drive_to_next_grid_line(label, expected_color, next_is_corner=False):
        """Drive straight until the color sensor sees the next grid line."""
        show_step("Line drive", label, expected_color)

        car.steer_center()
        await asyncio.sleep_ms(300)

        # Starting inside a square should show background first. If the sensor
        # starts on a line, this waits until it leaves that line before counting
        # the next line crossing.
        saw_background = False
        sane_line_count = 0
        start_ms = time.ticks_ms()

        car.enable_imu_reference(True, reset_reference=True)
        car.drive(DRIVE_POWER, CENTER_ANGLE)

        try:
            while True:
                car.drive(DRIVE_POWER, CENTER_ANGLE)

                elapsed_ms = time.ticks_diff(time.ticks_ms(), start_ms)
                rgb = floor_rgb()
                clear = None if rgb is None else int(rgb.get("clear", 0))
                line_seen = on_grid_line(clear)
                detected_color = detected_line_color(rgb) if line_seen else None

                if not line_seen:
                    saw_background = True
                    sane_line_count = 0
                elif saw_background and line_color_is_sane(detected_color, expected_color):
                    sane_line_count += 1
                    show_step(
                        "Line check",
                        "want {}".format(expected_color),
                        "{}/{}".format(sane_line_count, LINE_MATCH_CONFIRMATIONS),
                    )
                    if sane_line_count < LINE_MATCH_CONFIRMATIONS:
                        await asyncio.sleep_ms(LINE_CHECK_MS)
                        continue

                    center_ms = CORNER_LINE_CENTER_MS if next_is_corner else LINE_CENTER_MS
                    show_step("Line found", expected_color, "seen {}".format(detected_color))
                    await asyncio.sleep_ms(center_ms)
                    break
                elif saw_background:
                    sane_line_count = 0
                    show_step("Ignore line", "want {}".format(expected_color), "saw {}".format(detected_color))

                show_step("Line drive", expected_color, "clear {}".format(clear))

                if elapsed_ms >= LINE_TIMEOUT_MS:
                    show_step("Line drive", "timeout", label)
                    break

                await asyncio.sleep_ms(LINE_CHECK_MS)
        finally:
            await stop_and_center()

    async def turn_degrees(side, degrees, label):
        """Turn left or right until the IMU gyro adds up to the requested angle."""
        steering = RIGHT_TURN_ANGLE if side == "right" else LEFT_TURN_ANGLE
        show_step("{} turn".format(side), label, "{} deg".format(degrees))

        car.enable_imu_reference(False)
        car.drive(TURN_POWER, steering)

        turned_abs_deg = 0.0
        last_ms = time.ticks_ms()

        try:
            while turned_abs_deg < float(degrees):
                now_ms = time.ticks_ms()
                dt_s = time.ticks_diff(now_ms, last_ms) / 1000.0
                last_ms = now_ms

                turned_abs_deg += abs(gyro_z_dps()) * dt_s
                show_step(
                    "{} turn".format(side),
                    label,
                    "{:.1f}/{:.1f} deg".format(turned_abs_deg, degrees),
                )

                await asyncio.sleep_ms(LOOP_MS)
        finally:
            await stop_and_center()

        return turned_abs_deg

    async def drive_format_a_loop():
        """Follow the outer perimeter for three laps and stop at START_GRID."""
        current = START_GRID
        total_steps = len(OUTER_LOOP) * LAPS_TO_RUN
        turn_side = turn_side_for_direction()
        last_turn_deg = 0.0

        for step in range(1, total_steps + 1):
            target = next_square(current)
            label = "{}>{}".format(current, target)
            expected_color = expected_line_color(current, target)
            next_is_corner = target in CORNER_GRIDS

            show_step("Route", "step {}/{}".format(step, total_steps), expected_color)
            await drive_to_next_grid_line(label, expected_color, next_is_corner)

            current = target

            if current in CORNER_GRIDS:
                show_step("Corner", current, "score point")
                last_turn_deg = await turn_degrees(turn_side, TURN_90_DEG, current)

            if current == START_GRID:
                completed_laps = step // len(OUTER_LOOP)
                show_step("Lap complete", "{} of {}".format(completed_laps, LAPS_TO_RUN))

            await asyncio.sleep_ms(250)

        return last_turn_deg

    try:
        if START_GRID not in OUTER_LOOP:
            show_step("Bad START_GRID", START_GRID)
            return

        if DIRECTION not in ("clockwise", "counterclockwise"):
            show_step("Bad DIRECTION", DIRECTION)
            return

        show_step("Format A", START_GRID, DIRECTION)
        await stop_and_center()
        await wait_for_positioning()

        turned = await drive_format_a_loop()

        show_step(
            "Done",
            "{} laps complete".format(LAPS_TO_RUN),
            "last turn {:.1f}".format(turned),
        )

    finally:
        await stop_and_center()
