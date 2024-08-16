#!/usr/bin/env python

import Jetson.GPIO as GPIO, time


class decoder:
    """Class to decode magnetic rotary encoder pulses."""

    def __init__(self, gpioA, gpioB, callback):

        self.gpioA = gpioA
        self.gpioB = gpioB
        self.callback = callback

        self.levA = 0
        self.levB = 0

        self.lastGpio = None

        GPIO.setmode(GPIO.BCM)
        GPIO.setup([gpioA, gpioB], GPIO.IN)
        self.cbAr = GPIO.add_event_detect(self.gpioA, GPIO.RISING, callback=self._cb_A_r, bouncetime=None, polltime=1)
        self.cbAf = GPIO.add_event_detect(self.gpioA, GPIO.FALLING, callback=self._cb_A_f, bouncetime=None, polltime=1)
        self.cbBr = GPIO.add_event_detect(self.gpioB, GPIO.RISING, callback=self._cb_B_r, bouncetime=None, polltime=1)
        self.cbBf = GPIO.add_event_detect(self.gpioB, GPIO.FALLING, callback=self._cb_B_f, bouncetime=None, polltime=1)

    def _cb_A_r(self, gpio):
        self.levA = 1
        if self.levB == 1:
            self.callback(1)

    def _cb_A_f(self, gpio):
        self.levA = 0

    def _cb_B_r(self, gpio):
        self.levB = 1
        if self.levA == 1:
            self.callback(-1)

    def _cb_B_f(self, gpio):
        self.levB = 0

    def _pulse(self, gpio):

        """
        Decode the rotary encoder pulse.

                   +---------+         +---------+      0
                   |         |         |         |
         A         |         |         |         |
                   |         |         |         |
         +---------+         +---------+         +----- 1

             +---------+         +---------+            0
             |         |         |         |
         B   |         |         |         |
             |         |         |         |
         ----+         +---------+         +---------+  1
        """

        level = GPIO.input(gpio)

        if gpio == self.gpioA:
            self.levA = level
        else:
            self.levB = level

        if gpio != self.lastGpio:  # debounce
            self.lastGpio = gpio

            if gpio == self.gpioA and level == 1:
                if self.levB == 1:
                    self.callback(1)
            elif gpio == self.gpioB and level == 1:
                if self.levA == 1:
                    self.callback(-1)


if __name__ == "__main__":
    import time

    pos_l = 0
    pos_r = 0

    def callback_l(way):
        global pos_l
        pos_l += way
        print("pos_l={}".format(pos_l))

    def callback_r(way):
        global pos_r
        pos_r += way
        print("pos_r={}".format(pos_r))

    decoder_l = decoder(22, 10, callback_l)
    decoder_r = decoder( 9, 11, callback_r)

    print("runnung")

    try:
        while True:
            pass
    finally:
        GPIO.cleanup()  # cleanup all GPIOs
        print("endung")
