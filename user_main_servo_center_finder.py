# Zebra SDV: Servo Center Finder
# Flash this file as user_main.py when you want to tune CENTER_ANGLE.
#
# What this program does:
# 1. Waits 10 seconds so you can place the robot on a clear floor.
# 2. Tests several steering angles near START_CENTER_ANGLE.
# 3. Tests both positive and negative motor power, because some gearing makes
#    negative motor power drive the robot forward.
# 4. Drives slowly for each angle and uses the IMU gyro to measure yaw drift.
# 5. Displays and prints the angle and motor sign that drove the straightest.
#
# Important:
# The robot will move during this test. Put it on a clear, safe area.

import time
import uasyncio as asyncio
from robot.ackermann import AckermannDrive


def show_step(zbot, line1, line2="", line3="", line4=""):
    """Show progress on the robot display and serial console."""
    zbot.display(line1, line2, line3, line4)
    print("CENTER:", line1, line2, line3, line4)


# -------------------------
# Student setup values
# -------------------------

# Match these ports to the real robot wiring.
DRIVE_MOTOR_PORT = 2
STEERING_PORT = 1

# The search checks angles around this starting guess.
START_CENTER_ANGLE = 90

# Search size. With the defaults, this checks:
# 78, 82, 86, 90, 94, 98, 102
SEARCH_SPAN_DEG = 12
SEARCH_STEP_DEG = 4

# Slow power is safer and makes the yaw measurement less jumpy.
# The program tests both signs. Watch the robot to see which sign is forward
# for your drivetrain, then use that signed value in your challenge program.
TEST_DRIVE_POWERS = (25, -25)

# How long to drive for each angle.
TEST_MS = 1200

# Time between tests so the robot settles.
SETTLE_MS = 600

# Startup positioning delay.
START_DELAY_S = 10

# IMU sampling.
LOOP_MS = 20
GYRO_DEADBAND_DPS = 0.8


async def main(zbot):
    car = AckermannDrive(
        zbot,
        drive_motor_port=DRIVE_MOTOR_PORT,
        steering_port=STEERING_PORT,
        center_angle=START_CENTER_ANGLE,
        min_angle=45,
        max_angle=135,
        imu_ref=False,
    )

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

    async def stop_and_settle():
        """Stop the robot and give it a moment to settle."""
        car.stop()
        car.steer_center()
        await asyncio.sleep_ms(SETTLE_MS)

    async def wait_for_positioning():
        """Give the driver time to place the robot before testing starts."""
        for seconds_left in range(START_DELAY_S, 0, -1):
            show_step(zbot, "Center finder", "starts in", "{} sec".format(seconds_left))
            await asyncio.sleep_ms(1000)

    def build_test_angles():
        """Build a list of steering angles to test, centered around the guess."""
        angles = []
        start = START_CENTER_ANGLE - SEARCH_SPAN_DEG
        stop = START_CENTER_ANGLE + SEARCH_SPAN_DEG

        angle = start
        while angle <= stop:
            angles.append(angle)
            angle += SEARCH_STEP_DEG

        return angles

    async def measure_yaw_for_angle(angle, drive_power):
        """Drive at one steering angle and return total yaw drift in degrees."""
        show_step(
            zbot,
            "Testing angle",
            "power {}".format(drive_power),
            "angle {}".format(angle),
        )

        car.steer(angle)
        await asyncio.sleep_ms(250)
        car.drive(drive_power, angle)

        yaw_deg = 0.0
        start_ms = time.ticks_ms()
        last_ms = start_ms

        try:
            while time.ticks_diff(time.ticks_ms(), start_ms) < TEST_MS:
                now_ms = time.ticks_ms()
                dt_s = time.ticks_diff(now_ms, last_ms) / 1000.0
                last_ms = now_ms

                yaw_deg += gyro_z_dps() * dt_s

                show_step(
                    zbot,
                    "Power {}".format(drive_power),
                    "angle {}".format(angle),
                    "yaw {:.1f}".format(yaw_deg),
                )

                await asyncio.sleep_ms(LOOP_MS)
        finally:
            await stop_and_settle()

        return yaw_deg

    async def find_best_center_for_power(drive_power):
        """Find the best steering center for one signed motor power."""
        best_angle = START_CENTER_ANGLE
        best_yaw_abs = None

        for angle in build_test_angles():
            yaw_deg = await measure_yaw_for_angle(angle, drive_power)
            yaw_abs = abs(yaw_deg)

            show_step(
                zbot,
                "Angle result",
                "power {} angle {}".format(drive_power, angle),
                "yaw {:.1f}".format(yaw_deg),
            )
            print(
                "CENTER_RESULT power={} angle={} yaw={:.3f}".format(
                    drive_power,
                    angle,
                    yaw_deg,
                )
            )

            if best_yaw_abs is None or yaw_abs < best_yaw_abs:
                best_angle = angle
                best_yaw_abs = yaw_abs

        return best_angle, best_yaw_abs

    async def find_best_motor_and_center():
        """Try both motor signs and return the best power and center angle."""
        best_power = TEST_DRIVE_POWERS[0]
        best_angle = START_CENTER_ANGLE
        best_yaw_abs = None

        for drive_power in TEST_DRIVE_POWERS:
            show_step(zbot, "Motor sign test", "power {}".format(drive_power))

            angle, yaw_abs = await find_best_center_for_power(drive_power)

            show_step(
                zbot,
                "Power result",
                "power {}".format(drive_power),
                "center {}".format(angle),
            )
            print(
                "POWER_RESULT power={} best_angle={} yaw_abs={:.3f}".format(
                    drive_power,
                    angle,
                    yaw_abs,
                )
            )

            if best_yaw_abs is None or yaw_abs < best_yaw_abs:
                best_power = drive_power
                best_angle = angle
                best_yaw_abs = yaw_abs

        return best_power, best_angle, best_yaw_abs

    try:
        show_step(zbot, "Servo center", "finder ready")
        await stop_and_settle()
        await wait_for_positioning()

        best_power, best_angle, best_yaw_abs = await find_best_motor_and_center()

        show_step(
            zbot,
            "Best result",
            "power {} center {}".format(best_power, best_angle),
            "yaw {:.1f}".format(best_yaw_abs),
        )
        zbot.display(
            "Use these:",
            "DRIVE_POWER={}".format(best_power),
            "CENTER_ANGLE={}".format(best_angle),
            "yaw {:.1f}".format(best_yaw_abs),
        )
        print(
            "CENTER_BEST power={} angle={} yaw_abs={:.3f}".format(
                best_power,
                best_angle,
                best_yaw_abs,
            )
        )

        # Leave the steering servo at the suggested center angle.
        car.stop()
        car.steer(best_angle)

    finally:
        car.stop()
