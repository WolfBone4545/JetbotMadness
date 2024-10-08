#!/usr/bin/python3

import time
import math
import smbus


class RobotMotors:

    def __init__(self, pwm_freq=500, debug=False):
        self.pwm = PCA9685(0x40, debug)
        self.pwm.setPWMFreq(pwm_freq)
        self.PWMA = 0
        self.AIN1 = 1
        self.AIN2 = 2
        self.PWMB = 5
        self.BIN1 = 3
        self.BIN2 = 4

    def motorRun(self, motor, index, speed):
        if speed > 100:
            return
        if (motor == 0):
            self.pwm.setDutycycle(self.PWMA, speed)
            if ((index == "forward") or (index == "dopredu")):
                self.pwm.setLevel(self.AIN1, 0)
                self.pwm.setLevel(self.AIN2, 1)
            elif ((index == "backward") or (index == "dozadu")):
                self.pwm.setLevel(self.AIN1, 1)
                self.pwm.setLevel(self.AIN2, 0)
            else:
                print("Unkwnown direction motor A")
        elif (motor == 1):
            self.pwm.setDutycycle(self.PWMB, speed)
            if ((index == "forward") or (index == "dopredu")):
                self.pwm.setLevel(self.BIN1, 1)
                self.pwm.setLevel(self.BIN2, 0)
            elif ((index == "backward") or (index == "dozadu")):
                self.pwm.setLevel(self.BIN1, 0)
                self.pwm.setLevel(self.BIN2, 1)
            else:
                print("Unkwnown direction motor B")
        else:
            print("The robot has only two motors")

    def motorStop(self, motor):
        if (motor == 0):
            self.pwm.setDutycycle(self.PWMA, 0)
        elif (motor == 1):
            self.pwm.setDutycycle(self.PWMB, 1)
        else:
            print("The roboy has only two motors")

    def goForward(self, speed):
        self.motorRun(0, "forward", speed)
        self.motorRun(1, "forward", speed)

    def goBackward(self, speed):
        self.motorRun(0, "backward", speed)
        self.motorRun(1, "backward", speed)

    def goLeft(self, speed_left, speed_right):
        self.motorRun(0, "forward", speed_left)
        self.motorRun(1, "backward", speed_right)

    def goRight(self, speed_left, speed_right):
        self.motorRun(0, "backward", speed_left)
        self.motorRun(1, "forward", speed_right)

    def stop(self):
        self.motorStop(0)
        self.motorStop(1)


# ============================================================================
# Raspi PCA9685 16-Channel PWM Servo Driver
# ============================================================================

class PCA9685:
    # Registers/etc.
    __SUBADR1 = 0x02
    __SUBADR2 = 0x03
    __SUBADR3 = 0x04
    __MODE1 = 0x00
    __PRESCALE = 0xFE
    __LED0_ON_L = 0x06
    __LED0_ON_H = 0x07
    __LED0_OFF_L = 0x08
    __LED0_OFF_H = 0x09
    __ALLLED_ON_L = 0xFA
    __ALLLED_ON_H = 0xFB
    __ALLLED_OFF_L = 0xFC
    __ALLLED_OFF_H = 0xFD

    def __init__(self, address, debug=False):
        self.bus = smbus.SMBus(1)
        self.address = address
        self.debug = debug
        if (self.debug):
            print("Reseting PCA9685")
        self.write(self.__MODE1, 0x00)

    def write(self, reg, value):
        "Writes an 8-bit value to the specified register/address"
        self.bus.write_byte_data(self.address, reg, value)
        if (self.debug):
            print("I2C: Write 0x%02X to register 0x%02X" % (value, reg))

    def read(self, reg):
        "Read an unsigned byte from the I2C device"
        result = self.bus.read_byte_data(self.address, reg)
        if (self.debug):
            print("I2C: Device 0x%02X returned 0x%02X from reg 0x%02X" % (self.address, result & 0xFF, reg))
        return result

    def setPWMFreq(self, freq):
        "Sets the PWM frequency"
        prescaleval = 25000000.0  # 25MHz
        prescaleval /= 4096.0  # 12-bit
        prescaleval /= float(freq)
        prescaleval -= 1.0
        if (self.debug):
            print("Setting PWM frequency to %d Hz" % freq)
            print("Estimated pre-scale: %d" % prescaleval)
        prescale = math.floor(prescaleval + 0.5)
        if (self.debug):
            print("Final pre-scale: %d" % prescale)

        oldmode = self.read(self.__MODE1);
        newmode = (oldmode & 0x7F) | 0x10  # sleep
        self.write(self.__MODE1, newmode)  # go to sleep
        self.write(self.__PRESCALE, int(math.floor(prescale)))
        self.write(self.__MODE1, oldmode)
        time.sleep(0.005)
        self.write(self.__MODE1, oldmode | 0x80)

    def setPWM(self, channel, on, off):
        "Sets a single PWM channel"
        self.write(self.__LED0_ON_L + 4 * channel, on & 0xFF)
        self.write(self.__LED0_ON_H + 4 * channel, on >> 8)
        self.write(self.__LED0_OFF_L + 4 * channel, off & 0xFF)
        self.write(self.__LED0_OFF_H + 4 * channel, off >> 8)
        if (self.debug):
            print("channel: %d  LED_ON: %d LED_OFF: %d" % (channel, on, off))

    def setDutycycle(self, channel, pulse):
        self.setPWM(channel, 0, int(pulse * (4096 / 100)))

    def setLevel(self, channel, value):
        if (value == 1):
            self.setPWM(channel, 0, 4095)
        else:
            self.setPWM(channel, 0, 0)


if __name__ == "__main__":

    # pwm = PCA9685(0x5f, debug=False)
    # pwm.setPWMFreq(50)
    # pwm.setDutycycle(0,100)
    # pwm.setLevel(1,0)
    # pwm.setLevel(2,1)

    motors = RobotMotors()

    try:
        while True:
            motors.goForward(50)
            # motors.goBackward(50)
            # motors.goLeft(50, 50)
            # motors.goRight(50, 50)
    except KeyboardInterrupt:
        motors.goForward(0)
