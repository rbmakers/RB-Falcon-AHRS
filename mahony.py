import math

class Mahony:

    def __init__(self, Kp=0.5, Ki=0.0, sample_freq=100):
        self._Kp = Kp
        self._Ki = Ki
        self.twoKp = 2.0 * Kp  # 2 * proportional gain (Kp)
        self.twoKi = 2.0 * Ki  # 2 * integral gain (Ki)
        self.q0 = 1.0
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0
        self.integralFBx = 0.0
        self.integralFBy = 0.0
        self.integralFBz = 0.0

        self._sample_freq = sample_freq
        self.invSampleFreq = 1.0 / self.sample_freq

        self._roll = 0.0
        self._yaw = 0.0
        self._pitch = 0.0
        self._anglesComputed = False

    def _inv_sqrt(self, x):
        return x ** -0.5

    def update(self, gx, gy, gz, ax, ay, az, mx, my, mz):
        """Call this function sample_freq times a second with values from your sensor
        The units of the accelerometer and magnetometer do not matter for this alogirthm
        The gryoscope must be in degrees/sec

        :param gx, gy, gz: Gyroscope values in degrees/sec
        :param ax, ay, az: Accelerometer values
        :param mx, my, mz: Magnetometer values
        """

        recipNorm = 0
        q0q0 = q0q1 = q0q2 = q0q3 = q1q1 = q1q2 = q1q3 = q2q2 = q2q3 = q3q3 = 0
        hx = hy = bx = bz = 0
        halfvx = halfvy = halfvz = halfwx = halfwy = halfwz = 0
        halfex = halfey = halfez = 0
        qa = qb = qc = 0

        # Use IMU algorithm if magnetometer measurement invalid
        # (avoids NaN in magnetometer normalisation)
        if (mx == 0.0) and (my == 0.0) and (mz == 0.0):
            self.update_IMU(gx, gy, gz, ax, ay, az)
            return

        # Convert gyroscope degrees/sec to radians/sec
        gx *= 0.0174533
        gy *= 0.0174533
        gz *= 0.0174533

        # Compute feedback only if accelerometer measurement valid
        # (avoids NaN in accelerometer normalisation)
        if not ((ax == 0.0) and (ay == 0.0) and (az == 0.0)):
            # Normalise accelerometer measurement
            recipNorm = self._inv_sqrt(ax * ax + ay * ay + az * az)
            ax *= recipNorm
            ay *= recipNorm
            az *= recipNorm

            # Normalise magnetometer measurement
            recipNorm = self._inv_sqrt(mx * mx + my * my + mz * mz)
            mx *= recipNorm
            my *= recipNorm
            mz *= recipNorm

            # Auxiliary variables to avoid repeated arithmetic
            q0q0 = self.q0 * self.q0
            q0q1 = self.q0 * self.q1
            q0q2 = self.q0 * self.q2
            q0q3 = self.q0 * self.q3
            q1q1 = self.q1 * self.q1
            q1q2 = self.q1 * self.q2
            q1q3 = self.q1 * self.q3
            q2q2 = self.q2 * self.q2
            q2q3 = self.q2 * self.q3
            q3q3 = self.q3 * self.q3

            # Reference direction of Earth's magnetic field
            hx = 2.0 * (
                mx * (0.5 - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2)
            )
            hy = 2.0 * (
                mx * (q1q2 + q0q3) + my * (0.5 - q1q1 - q3q3) + mz * (q2q3 - q0q1)
            )
            bx = math.sqrt(hx * hx + hy * hy)
            bz = 2.0 * (
                mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5 - q1q1 - q2q2)
            )

            # Estimated direction of gravity and magnetic field
            halfvx = q1q3 - q0q2
            halfvy = q0q1 + q2q3
            halfvz = q0q0 - 0.5 + q3q3
            halfwx = bx * (0.5 - q2q2 - q3q3) + bz * (q1q3 - q0q2)
            halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3)
            halfwz = bx * (q0q2 + q1q3) + bz * (0.5 - q1q1 - q2q2)

            # Error is sum of cross product between estimated direction
            # and measured direction of field vectors
            halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy)
            halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz)
            halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx)

            # Compute and apply integral feedback if enabled
            if self.twoKi > 0.0:
                # integral error scaled by Ki
                self.integralFBx += self.twoKi * halfex * self.invSampleFreq
                self.integralFBy += self.twoKi * halfey * self.invSampleFreq
                self.integralFBz += self.twoKi * halfez * self.invSampleFreq
                gx += self.integralFBx  # apply integral feedback
                gy += self.integralFBy
                gz += self.integralFBz
            else:
                self.integralFBx = 0.0  # prevent integral windup
                self.integralFBy = 0.0
                self.integralFBz = 0.0

            # Apply proportional feedback
            gx += self.twoKp * halfex
            gy += self.twoKp * halfey
            gz += self.twoKp * halfez

        # Integrate rate of change of quaternion
        gx *= 0.5 * self.invSampleFreq  # pre-multiply common factors
        gy *= 0.5 * self.invSampleFreq
        gz *= 0.5 * self.invSampleFreq
        qa = self.q0
        qb = self.q1
        qc = self.q2
        self.q0 += -qb * gx - qc * gy - self.q3 * gz
        self.q1 += qa * gx + qc * gz - self.q3 * gy
        self.q2 += qa * gy - qb * gz + self.q3 * gx
        self.q3 += qa * gz + qb * gy - qc * gx

        # Normalise quaternion
        recipNorm = self._inv_sqrt(
            self.q0 * self.q0
            + self.q1 * self.q1
            + self.q2 * self.q2
            + self.q3 * self.q3
        )
        self.q0 *= recipNorm
        self.q1 *= recipNorm
        self.q2 *= recipNorm
        self.q3 *= recipNorm
        self._anglesComputed = False

    # -------------------------------------------------------------------------------------------
    # IMU algorithm update

    def update_IMU(self, gx, gy, gz, ax, ay, az):
        """
        Called is was have no mag reading (internal use)
        """
        recipNorm = 0
        halfvx = halfvy = halfvz = 0
        halfex = halfey = halfez = 0
        qa = qb = qc = 0

        # Convert gyroscope degrees/sec to radians/sec
        gx *= 0.0174533
        gy *= 0.0174533
        gz *= 0.0174533

        # Compute feedback only if accelerometer measurement valid
        # (avoids NaN in accelerometer normalisation)
        if not ((ax == 0.0) and (ay == 0.0) and (az == 0.0)):
            # Normalise accelerometer measurement
            recipNorm = self._inv_sqrt(ax * ax + ay * ay + az * az)
            ax *= recipNorm
            ay *= recipNorm
            az *= recipNorm

            # Estimated direction of gravity
            halfvx = self.q1 * self.q3 - self.q0 * self.q2
            halfvy = self.q0 * self.q1 + self.q2 * self.q3
            halfvz = self.q0 * self.q0 - 0.5 + self.q3 * self.q3

            # Error is sum of cross product between estimated
            # and measured direction of gravity
            halfex = ay * halfvz - az * halfvy
            halfey = az * halfvx - ax * halfvz
            halfez = ax * halfvy - ay * halfvx

            # Compute and apply integral feedback if enabled
            if self.twoKi > 0.0:
                # integral error scaled by Ki
                self.integralFBx += self.twoKi * halfex * self.invSampleFreq
                self.integralFBy += self.twoKi * halfey * self.invSampleFreq
                self.integralFBz += self.twoKi * halfez * self.invSampleFreq
                gx += self.integralFBx  # apply integral feedback
                gy += self.integralFBy
                gz += self.integralFBz
            else:
                self.integralFBx = 0.0  # prevent integral windup
                self.integralFBy = 0.0
                self.integralFBz = 0.0

            # Apply proportional feedback
            gx += self.twoKp * halfex
            gy += self.twoKp * halfey
            gz += self.twoKp * halfez

        # Integrate rate of change of quaternion
        gx *= 0.5 * self.invSampleFreq  # pre-multiply common factors
        gy *= 0.5 * self.invSampleFreq
        gz *= 0.5 * self.invSampleFreq
        qa = self.q0
        qb = self.q1
        qc = self.q2
        self.q0 += -qb * gx - qc * gy - self.q3 * gz
        self.q1 += qa * gx + qc * gz - self.q3 * gy
        self.q2 += qa * gy - qb * gz + self.q3 * gx
        self.q3 += qa * gz + qb * gy - qc * gx

        # Normalise quaternion
        recipNorm = self._inv_sqrt(
            self.q0 * self.q0
            + self.q1 * self.q1
            + self.q2 * self.q2
            + self.q3 * self.q3
        )
        self.q0 *= recipNorm
        self.q1 *= recipNorm
        self.q2 *= recipNorm
        self.q3 *= recipNorm
        self._anglesComputed = False

    def compute_angles(self):
        """
        Compute all the angles if there have been new samples (internal use)
        """
        self._roll = math.atan2(
            self.q0 * self.q1 + self.q2 * self.q3,
            0.5 - self.q1 * self.q1 - self.q2 * self.q2,
        )
        self._pitch = math.asin(-2.0 * (self.q1 * self.q3 - self.q0 * self.q2))
        self._yaw = math.atan2(
            self.q1 * self.q2 + self.q0 * self.q3,
            0.5 - self.q2 * self.q2 - self.q3 * self.q3,
        )
        self._anglesComputed = True

    @property
    def yaw(self):
        """
        Current yaw (z-axis) value in radians/sec. (read-only)
        """
        if not self._anglesComputed:
            self.compute_angles()
        return self._yaw

    @property
    def pitch(self):
        """
        Current pitch (y-axis) value in radians/sec. (read-only)
        """
        if not self._anglesComputed:
            self.compute_angles()
        return self._pitch

    @property
    def roll(self):
        """
        Current roll (x-axis) value in radians/sec. (read-only)
        """
        if not self._anglesComputed:
            self.compute_angles()
        return self._roll

    @property
    def Kp(self):
        """The current Kp value (Proportional gain)."""
        return self._Kp

    @Kp.setter
    def Kp(self, value):
        self._Kp = value
        self.twoKp = 2.0 * self._Kp

    @property
    def Ki(self):
        """The current Ki value (Integral gain)."""
        return self._Ki

    @Ki.setter
    def Ki(self, value):
        self._Ki = value
        self.twoKi = 2.0 * self._Ki

    @property
    def sample_freq(self):
        """The current sample frequency value in Hertz."""
        return self._sample_freq

    @sample_freq.setter
    def sample_freq(self, value):
        self._sample_freq = value
        self.invSampleFreq = 1.0 / self.sample_freq
