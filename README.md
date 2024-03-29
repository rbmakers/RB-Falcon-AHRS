<picture>
  <source media="(prefers-color-scheme: dark)" srcset="https://github.com/rbmakers/RB-Falcon-AHRS/assets/148856699/b1bcbce7-7023-4aa2-89dd-63e37c55d587">
  <source media="(prefers-color-scheme: light)" srcset="https://github.com/rbmakers/RB-Falcon-AHRS/assets/148856699/b1bcbce7-7023-4aa2-89dd-63e37c55d587">
  <img alt="Shows an illustrated sun in light mode and a moon with stars in dark mode." src="https://github.com/rbmakers/RB-Falcon-AHRS/assets/148856699/b1bcbce7-7023-4aa2-89dd-63e37c55d587">
</picture>

                                                         
                                                          
RB-Falcon, capable of image recognition, is an all-in-one quadcopter flight controller using a STM32F765 MCU. 

Built-in sensors include IMU(LSM9DS1), Pressure&Temp(BMP280), and Image(OV7725). Also included chips on board are a NRF24L01 for remote control, a STC4054 for charging a Li-Ion battery, four 30V N-channel MOSFETs for coreless DC motors, a SD card holder for storage up to 32GB, and a RGB LED for the tutorial programming.

The programming language of RB-Falcon is MicroPython. RB-Falcon can be programmed using Thonny or OPENMV IDEs. MicroPython scripts in this repository are for demonstrating AHRS algorithms of Mahony and Madgwick.

Reference : https://github.com/gamblor21/Gamblor21_CircuitPython_AHRS/tree/master

Mahony Filter : https://hal.science/hal-00488376/document

Madgwick Filter : https://courses.cs.washington.edu/courses/cse466/14au/labs/l4/madgwick_internal_report.pdf


