# Set Pin Factory to pigpio https://gpiozero.readthedocs.io/en/stable/api_pins.html#changing-the-pin-factory
from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import Device, Servo
from gpiozero.tools import scaled_half
Device.pin_factory = PiGPIOFactory()

##### Taken from gpiozero source #####
from random import random
from time import sleep
from itertools import cycle
from math import sin, cos, pi, isclose
from statistics import mean

def sin_values(period=360):
    """
    Provides an infinite source of values representing a sine wave (from -1 to
    +1) which repeats every *period* values. For example, to produce a "siren"
    effect with a couple of LEDs that repeats once a second::

        from gpiozero import PWMLED
        from gpiozero.tools import sin_values, scaled_half, inverted
        from signal import pause

        red = PWMLED(2)
        blue = PWMLED(3)

        red.source_delay = 0.01
        blue.source_delay = red.source_delay
        red.source = scaled_half(sin_values(100))
        blue.source = inverted(red)

        pause()

    If you require a different range than -1 to +1, see :func:`scaled`.
    """
    angles = (2 * pi * i / period for i in range(period))
    for a in cycle(angles):
        yield sin(a)
##### End #####

servo = Servo(12, min_angle=-40, max_angle=40, min_pulse_width=0.0009, max_pulse_width=0.0019)

servo.source_delay = 0.1
servo.source = sin_values()

