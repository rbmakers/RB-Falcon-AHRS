RB-Falcon, capable of image recognition, is an all-in-one quadcopter flight controller using a STM32F765 MCU. 

Built-in sensors include IMU(LSM9DS1), Pressure&Temp(BMP280), and Image(OV7725). Also included chips on board are a NRF24L01 for remote control, a STC4054 for charging a Li-Ion battery, and four 30V N-channel MOSFETs for motors.

The programming language of RB-Falcon is MicroPython. RB-Falcon can be programmed using Thonny or OPENMV IDEs. MicroPython scripts in this repository are for demonstrating AHRS algorithms of Mahony and Madgwick.

Reference : https://github.com/gamblor21/Gamblor21_CircuitPython_AHRS/tree/master

Mahony Filter : https://hal.science/hal-00488376/document

Madgwick Filter : https://courses.cs.washington.edu/courses/cse466/14au/labs/l4/madgwick_internal_report.pdf


