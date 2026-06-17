# Zebra SDV: Ackermann Square Test, IMU Turn Only
# Flash this file as user_main.py when you want a square-driving test.
#
# What this program does:
# 1. Drives forward for one side of a square.
# 2. Uses the Zebra IMU gyro to measure a 90 degree right turn.
# 3. Repeats four sides per lap, for three laps.
#
# Why it is written this way:
# The flasher runtime currently does not expose zbot.reset_imu_distance()
# or zbot.imu_distance(), so this file avoids those calls completely.


async def main(zbot):
    import math
    import time
    import uasyncio as asyncio
    from robot.ackermann import AckermannDrive

    # -------------------------
    # Student setup values
    # -------------------------

    # Match these ports to the real robot wiring.
    DRIVE_MOTOR_PORT = 2
    STEERING_PORT = 1

    # Steering angles. Tune CENTER_ANGLE until the robot drives straight.
    # RIGHT_TURN_ANGLE is close to the max steering angle for a tighter turn.
    CENTER_ANGLE = 90
    RIGHT_TURN_ANGLE = 132

    # Motor power. Increase slowly if the robot needs more force.
    DRIVE_POWER = 35
    TURN_POWER = 30

    # This replaces the missing zbot.imu_distance() call.
    # Tune SIDE_DISTANCE_M until one straight side of the square is the length
    # you want. 1.0 meter is about 3.28 feet.
    SIDE_DISTANCE_M = 1.0

    # Square settings.
    SQUARE_LAPS = 3
    SIDES_PER_LAP = 4

    # The robot was underturning by about 5 degrees, so we ask the IMU loop for
    # 95 degrees to make the real-world turn closer to a square 90 degree turn.
    RIGHT_TURN_DEG = 95

    # Positioning delay before the robot starts driving.
    START_DELAY_S = 10

    # Loop timing.
    LOOP_MS = 20

    # Safety stop for a straight segment. Distance is still controlled by IMU,
    # but this prevents the robot from driving forever if the IMU is unplugged
    # or the estimate never reaches the target.
    MAX_FORWARD_MS = 12000

    # Ignore early IMU distance during motor startup. This prevents small hand
    # movements or launch vibration from being counted as real forward travel.
    IMU_DISTANCE_ARM_MS = 500

    # Small gyro readings are usually sensor noise, so ignore them.
    GYRO_DEADBAND_DPS = 1.0

    # IMU straight-line hold settings.
    # Bigger KP means the steering reacts harder when the robot drifts.
    # Bigger KD damps fast changes so the car is less likely to wiggle.
    # Bigger MAX_CORRECTION_DEG allows stronger steering corrections.
    # These values are intentionally moderate to avoid rapid steering shake.
    IMU_HOLD_KP = 1.1
    IMU_HOLD_KD = 0.08
    IMU_HOLD_MAX_CORRECTION_DEG = 18

    # IMU distance estimator settings.
    # Larger deadband ignores more vibration and hand movement. Smaller damping
    # lets speed build faster, but also lets drift build faster.
    GRAVITY_MPS2 = 9.80665
    IMU_DISTANCE_ACCEL_DEADBAND_MPS2 = 0.75
    IMU_DISTANCE_STILL_ACCEL_MPS2 = 0.90
    IMU_DISTANCE_STILL_GYRO_DPS = 3.0
    IMU_DISTANCE_STILL_MS = 500
    IMU_DISTANCE_MAX_DT_MS = 250
    IMU_DISTANCE_DAMPING = 0.94
    IMU_DISTANCE_MIN_SPEED_MPS = 0.04

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
        """Show the current code section on the robot display and serial console."""
        zbot.display(line1, line2, line3)
        print("SDV:", line1, line2, line3)

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

    class ImuDistanceEstimator:
        """Estimate distance by integrating IMU acceleration magnitude."""

        def __init__(self):
            self.reset()

        def reset(self):
            """Start a fresh distance estimate."""
            self.distance_m = 0.0
            self.raw_distance_m = 0.0
            self.speed_mps = 0.0
            self.accel_mps2 = 0.0
            self.accel_mag_g = 0.0
            self.gyro_mag_dps = 0.0
            self.still_ms = 0
            self.last_ms = None
            self.status = "reset"
            self.armed = False
            self.start_ms = time.ticks_ms()

        def reading_value(self, reading, keys):
            """Return the first numeric value found for a list of possible names."""
            for key in keys:
                value = reading.get(key)
                if isinstance(value, (int, float)):
                    return float(value)
            return None

        def update(self):
            """Read the IMU once and update distance, speed, and status."""
            imu = zbot.imu()
            reading = imu.get("value", {}) if imu else {}
            now_ms = time.ticks_ms()

            ax = self.reading_value(reading, ("ax_g", "accel_x_g", "ax", "accel_x"))
            ay = self.reading_value(reading, ("ay_g", "accel_y_g", "ay", "accel_y"))
            az = self.reading_value(reading, ("az_g", "accel_z_g", "az", "accel_z"))
            gx = self.reading_value(reading, ("gx_dps", "gyro_x_dps", "gx", "gyro_x")) or 0.0
            gy = self.reading_value(reading, ("gy_dps", "gyro_y_dps", "gy", "gyro_y")) or 0.0
            gz = self.reading_value(reading, ("gz_dps", "gyro_z_dps", "gz", "gyro_z")) or 0.0

            if ax is None or ay is None or az is None:
                self.status = "missing accel"
                return self

            if self.last_ms is None:
                self.last_ms = now_ms
                self.status = "waiting"
                return self

            dt_ms = time.ticks_diff(now_ms, self.last_ms)
            self.last_ms = now_ms

            if dt_ms <= 0:
                return self

            if dt_ms > IMU_DISTANCE_MAX_DT_MS:
                self.speed_mps = 0.0
                self.still_ms = 0
                self.status = "waiting"
                return self

            elapsed_ms = time.ticks_diff(now_ms, self.start_ms)
            if elapsed_ms < IMU_DISTANCE_ARM_MS:
                self.speed_mps = 0.0
                self.raw_distance_m = 0.0
                self.distance_m = 0.0
                self.status = "arming"
                return self

            self.armed = True

            self.accel_mag_g = math.sqrt((ax * ax) + (ay * ay) + (az * az))
            self.accel_mps2 = abs(self.accel_mag_g - 1.0) * GRAVITY_MPS2

            if self.accel_mps2 < IMU_DISTANCE_ACCEL_DEADBAND_MPS2:
                self.accel_mps2 = 0.0

            self.gyro_mag_dps = max(abs(gx), abs(gy), abs(gz))
            dt_s = dt_ms / 1000.0

            if (
                self.accel_mps2 < IMU_DISTANCE_STILL_ACCEL_MPS2
                and self.gyro_mag_dps < IMU_DISTANCE_STILL_GYRO_DPS
            ):
                self.still_ms += dt_ms
                self.speed_mps *= IMU_DISTANCE_DAMPING

                if self.still_ms >= IMU_DISTANCE_STILL_MS:
                    self.speed_mps = 0.0
            else:
                self.still_ms = 0
                self.speed_mps = (
                    self.speed_mps + (self.accel_mps2 * dt_s)
                ) * IMU_DISTANCE_DAMPING

            if self.accel_mps2 == 0.0 and self.speed_mps < IMU_DISTANCE_MIN_SPEED_MPS:
                self.speed_mps = 0.0

            self.raw_distance_m += self.speed_mps * dt_s
            self.distance_m = self.raw_distance_m
            self.status = "ok"
            return self

    async def stop_and_center():
        """Stop the drive motor and point the steering straight ahead."""
        car.stop()
        car.steer_center()
        await asyncio.sleep_ms(250)

    async def wait_for_positioning():
        """Give the driver time to position the robot before it starts."""
        for seconds_left in range(START_DELAY_S, 0, -1):
            show_step("Position robot", "starts in", "{} sec".format(seconds_left))
            await asyncio.sleep_ms(1000)

        show_step("Starting", "drive square")
        await asyncio.sleep_ms(300)

    async def drive_forward_distance(target_m, lap, side):
        """Drive straight until the local IMU distance estimate reaches target_m."""
        show_step(
            "Forward",
            "lap {} side {}".format(lap, side),
            "{:.2f} m".format(target_m),
        )

        car.steer_center()
        await asyncio.sleep_ms(300)

        # AckermannDrive can use the IMU to help hold the steering reference.
        car.enable_imu_reference(True, reset_reference=True)
        car.drive(DRIVE_POWER, CENTER_ANGLE)

        distance = ImuDistanceEstimator()
        start_ms = time.ticks_ms()
        try:
            while distance.distance_m < float(target_m):
                # Re-send a straight drive command each loop so the Ackermann
                # helper keeps applying IMU steering corrections.
                car.drive(DRIVE_POWER, CENTER_ANGLE)
                distance.update()

                elapsed_ms = time.ticks_diff(time.ticks_ms(), start_ms)
                show_step(
                    "Forward",
                    "L{} S{}".format(lap, side),
                    "{:.2f}/{:.2f} m".format(distance.distance_m, target_m),
                )

                if elapsed_ms >= MAX_FORWARD_MS:
                    show_step("Forward", "safety stop", distance.status)
                    break

                await asyncio.sleep_ms(LOOP_MS)
        finally:
            await stop_and_center()

    async def turn_right_degrees(degrees, lap, side):
        """Turn right until the IMU gyro adds up to the requested angle."""
        show_step(
            "Right turn",
            "lap {} side {}".format(lap, side),
            "target {} deg".format(degrees),
        )

        car.enable_imu_reference(False)
        car.drive(TURN_POWER, RIGHT_TURN_ANGLE)

        turned_abs_deg = 0.0
        last_ms = time.ticks_ms()

        try:
            while turned_abs_deg < float(degrees):
                now_ms = time.ticks_ms()
                dt_s = time.ticks_diff(now_ms, last_ms) / 1000.0
                last_ms = now_ms

                # abs() lets this work even if the gyro direction is reversed.
                turned_abs_deg += abs(gyro_z_dps()) * dt_s

                show_step(
                    "Right turn",
                    "L{} S{}".format(lap, side),
                    "{:.1f}/{:.1f} deg".format(turned_abs_deg, degrees),
                )

                await asyncio.sleep_ms(LOOP_MS)
        finally:
            await stop_and_center()

        return turned_abs_deg

    async def drive_square_laps():
        """Drive around a square several times."""
        last_turn_deg = 0.0

        for lap in range(1, SQUARE_LAPS + 1):
            show_step("Square lap", "{} of {}".format(lap, SQUARE_LAPS))

            for side in range(1, SIDES_PER_LAP + 1):
                await drive_forward_distance(SIDE_DISTANCE_M, lap, side)
                await asyncio.sleep_ms(250)

                last_turn_deg = await turn_right_degrees(RIGHT_TURN_DEG, lap, side)
                await asyncio.sleep_ms(250)

        return last_turn_deg

    try:
        show_step("Ackermann", "square x{}".format(SQUARE_LAPS), "IMU turns")
        await stop_and_center()
        await wait_for_positioning()

        turned = await drive_square_laps()

        show_step(
            "Done",
            "{} square laps".format(SQUARE_LAPS),
            "last {:.1f} deg".format(turned),
        )

    finally:
        await stop_and_center()
