import time
from machine import I2C
import lsm9ds1
import mahony

MAG_MIN = [-0.122803, -0.153442, -0.114502]
MAG_MAX = [-0.0638428, -0.110718, -0.0939941]

## Used to calibrate the magenetic sensor
def map_range(x, in_min, in_max, out_min, out_max):
    """
    Maps a number from one range to another.
    :return: Returns value mapped to new range
    :rtype: float
    """
    mapped = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    if out_min <= out_max:
        return max(min(mapped, out_max), out_min)

    return min(max(mapped, out_max), out_min)


## create the ahrs_filter
ahrs_filter = mahony.Mahony(50, 5, 100)

bus = I2C(2)
sensor = lsm9ds1.LSM9DS1(bus)

count = 0  # used to count how often we are feeding the ahrs_filter
lastPrint = time.ticks_ms()/1000
timestamp = time.ticks_us()

while True:
    
    if (time.ticks_us() - timestamp) > 10000: # 10000us = 10ms = 100Hz

        # read the magenetic sensor
        mx, my, mz = sensor.read_magnet()

        # adjust for magnetic calibration - hardiron only
        # calibration varies per device and physical location
        mx = map_range(mx, MAG_MIN[0], MAG_MAX[0], -1, 1)
        my = map_range(my, MAG_MIN[1], MAG_MAX[1], -1, 1)
        mz = map_range(mz, MAG_MIN[2], MAG_MAX[2], -1, 1)

        # read the gyroscope

        gx, gy, gz = sensor.read_gyro()
        # adjust for my gyro calibration values
        # calibration varies per device and physical location
        #gx -= 0
        #gy -= 0
        #gz += 0

        # read the accelerometer

        ax, ay, az = sensor.read_accel()

        # update the ahrs_filter with the values
        # gz and my are negative based on my installation
        ahrs_filter.update(gx, gy, -gz, ax, ay, az, mx, -my, mz)

        count += 1
        timestamp = time.ticks_us()

    # every 0.1 seconds print the ahrs_filter values
    if (time.ticks_ms()/1000) > lastPrint + 0.1:
        # ahrs_filter values are in radians/sec multiply by 57.20578 to get degrees/sec
        yaw = ahrs_filter.yaw * 57.20578
        if yaw < 0:  # adjust yaw to be between 0 and 360
            yaw += 360
        print(
            "Orientation: ",
            yaw,
            ", ",
            ahrs_filter.pitch * 57.29578,
            ", ",
            ahrs_filter.roll * 57.29578,
        )

        # print("Count: ", count)  
        count = 0                           # reset count
        lastPrint = time.ticks_ms()/1000
     
