# Zebra SDV: Default User Main
# Change this import to pick the SDV section you want to flash.

# Options:
# from user_main_sensor_check import main
# from user_main_ackermann_6ft_right_turn_imu_only import main
# from user_main_format_a_solo import main
# from user_main_format_a_solo_imu_only import main
# from user_main_format_b_alliance import main

# Current selection runs Format A with timed straight drives and IMU turns.
from user_main_format_a_solo_imu_only import main
