import math, time

import i2c

PCA9685_MODE1 = 0x0
PCA9685_PRESCALE = 0xFE

PCA9685_SLEEP = 0x10
PCA9685_RESTART = 0x80

SERVO_L = 0x8
SERVO_H = 0x9

SERVO_MULTIPLIER = 4

class ServoError(Exception):
    pass


class PwmOutOfBoundsError(ServoError):
    pass


class ServoController(object):

    def __init__(self, i2c_dev, freq = 60, pw_min=300, pw_max=600):
        self.i2c_dev = i2c_dev
        prescale = ServoController.calc_prescale(freq)

        self.init_i2c(prescale)

        self.servos = []

        for idx in range(16):
            servo = Servo(self.i2c_dev, idx, pw_min, pw_max)
            self.servos.append(servo)

    def init_i2c(self, prescale):
        old_mode = self.i2c_dev[PCA9685_MODE1]
        self.i2c_dev[PCA9685_MODE1] = (old_mode & 0x7F) | PCA9685_SLEEP # set sleep mode.
        self.i2c_dev[PCA9685_PRESCALE] = prescale
        self.i2c_dev[PCA9685_MODE1] = old_mode
        time.sleep(0.01)
        self.i2c_dev[PCA9685_MODE1] = (old_mode | PCA9685_RESTART) &  0b11101111

    @classmethod
    def calc_prescale(cls, freq):
        prescale = 25000000.0
        prescale /= 4096
        prescale /= freq
        prescale -= 1
        prescale += 0.5
        return math.floor(prescale)

class Servo(object):

    def __init__(self, i2c_dev, servo_num, pw_min=300, pw_max=600):
        self.i2c_dev = i2c_dev
        self.servo_num = servo_num

        self.pw_min = pw_min
        self.pw_max = pw_max

        self.low_addr = SERVO_L + (servo_num * SERVO_MULTIPLIER)
        self.high_addr = SERVO_H + (servo_num * SERVO_MULTIPLIER)

    def set_pwm(self, pwm):
        if pwm < self.pw_min or pwm > self.pw_max:
            raise PwmOutOfBoundsError("Requested PWM of %d" % pwm)

        self.i2c_dev[self.low_addr] = pwm & 0xff
        self.i2c_dev[self.high_addr] = pwm >> 8

