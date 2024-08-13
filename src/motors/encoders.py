#!/usr/bin/env python

from jetbot import GPIO


class decoder:
    """Class to decode magnetic rotary encoder pulses."""

    def __init__(self, gpioA, gpioB, callback):

        self.gpioA = gpioA
        self.gpioB = gpioB
        self.callback = callback

        self.levA = 0
        self.levB = 0

        self.lastGpio = None

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup([gpioA, gpioB], GPIO.IN)
        self.cbA = GPIO.add_event_detect(gpioA, GPIO.BOTH, callback=self._pulse, bouncetime=10, polltime=0.2)
        self.cbB = GPIO.add_event_detect(gpioB, GPIO.BOTH, callback=self._pulse, bouncetime=10, polltime=0.2)

    def _pulse(self, gpio, level, tick):

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

    decoder_l = rotary_encoder.decoder(22, 10, callback_l)
    decoder_r = rotary_encoder.decoder( 9, 11, callback_r)

    try:
        while True:
            time.sleep(1)
    finally:
        GPIO.cleanup()  # cleanup all GPIOs
