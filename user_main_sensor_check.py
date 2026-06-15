# Zebra SDV: Sensor Check
# Flash this file as user_main.py before tuning on the game mat.
#
# This file is standalone. It uses only the zbot object passed into main().

import uasyncio as asyncio


def show_step(zbot, section, detail="", extra="", more=""):
    """Show the current program section on the display and serial console."""
    zbot.display(section, detail, extra, more)
    print("SDV:", section, detail, extra, more)


# Downward color sensor used to inspect the mat.
FLOOR_COLOR_PORT = 2

# Front time-of-flight distance sensor used to check wall or obstacle distance.
# Use None if this sensor is not installed.
FRONT_TOF_PORT = 1


def gyro_z_dps(zbot):
    """Read gyro Z from the Zebra IMU, if it is available."""
    payload = zbot.imu()
    if isinstance(payload, dict) and isinstance(payload.get("value"), dict):
        payload = payload["value"]
    if not isinstance(payload, dict):
        return None

    for key in ("gz_dps", "gyro_z_dps", "gz", "gyro_z"):
        value = payload.get(key)
        if isinstance(value, (int, float)):
            return float(value)
    return None


async def main(zbot):
    show_step(zbot, "Sensor check", "starting")
    await asyncio.sleep_ms(800)

    while True:
        color = None
        rgb = None
        distance = None
        clear = None

        if FLOOR_COLOR_PORT is not None:
            color = zbot.color(FLOOR_COLOR_PORT)
            rgb = zbot.rgb(FLOOR_COLOR_PORT)
            if rgb is not None:
                clear = rgb.get("clear")

        if FRONT_TOF_PORT is not None:
            distance = zbot.tof(FRONT_TOF_PORT)

        zbot.display(
            "Sensor check",
            "Color {}".format(color),
            "Clear {}".format(clear),
            "ToF {}".format(distance),
        )
        print("SDV: Sensor check color={} clear={} tof={} gyroz={}".format(
            color,
            clear,
            distance,
            gyro_z_dps(zbot),
        ))

        await asyncio.sleep_ms(250)
