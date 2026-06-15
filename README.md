# SDV Standalone User Main Files

These files are starter programs for the STRIPE Self-Driving Vehicle Challenge.
Each challenge file is written as a standalone Zebra `user_main.py` example.

## Files

- `user_main_sensor_check.py`: displays sensor readings for mat tuning.
- `user_main_format_a_solo.py`: Format A, three-lap solo loop challenge.
- `user_main_format_b_alliance.py`: Format B, trailer pickup and delivery challenge.
- `user_main.py`: small selector file. Change its import to pick which program to flash.

## Dependency Pattern

The two driving programs use only one robot helper import:

```python
from robot.ackermann import AckermannDrive
```

There is no shared `sdv_config.py` setup file. Edit the constants near the top of
the standalone file you are flashing.

## Before Running

1. Pick the standalone file for the challenge section.
2. Tune `DRIVE_MOTOR_PORT`, `STEERING_PORT`, `CENTER_ANGLE`, `LEFT_TURN_ANGLE`, and `RIGHT_TURN_ANGLE`.
3. Tune `CELL_MS` until one grid-cell move is correct.
4. Tune `TURN_90_MS` until one turn is close to 90 degrees.
5. Set the challenge choices, such as `START_GRID`, `DIRECTION`, `TRAILER_SIDE`, and `DELIVERIES`.
6. Flash the chosen file as `user_main.py`, or change the import inside the included `user_main.py` selector.

## Sensor-Assisted Driving

The driving files now prefer sensor help when it is available:

- `USE_IMU_TURNS = True` uses the Zebra IMU gyro to stop turns near 90 degrees.
- `USE_COLOR_LINES = True` uses a downward color sensor to count the colored grid lines on the mat.
- `LINE_CLEAR_MAX` is the clear-channel threshold for detecting a grid line. Use `user_main_sensor_check.py` to read the clear value over white mat areas, colored lines, and black intersections.
- If a sensor is not installed or not tuned yet, set that feature to `False` and use the timing values.
