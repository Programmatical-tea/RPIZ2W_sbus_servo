# Set Pin Factory to pigpio https://gpiozero.readthedocs.io/en/stable/api_pins.html#changing-the-pin-factory
from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import Device, AngularServo
from gpiozero.tools import scaled
from signal import pause

Device.pin_factory = PiGPIOFactory()

##### Taken from gpiozero source #####
from random import random
from time import sleep
from itertools import cycle
from math import sin, cos, pi, isclose
from statistics import mean

import zmq

SERVO_GPIO_PIN_1 = 12

class Servo1:

    def __init__(self):
        self.A = 0
        self.B = 0
        self.period = 360
        self.servo1 = AngularServo(SERVO_GPIO_PIN_1,min_angle=-40, max_angle=40, min_pulse_width=0.0009, max_pulse_width=0.0019)
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PULL)
        self.socket.connect("tcp://127.0.0.1:7777")

    def update(self):
        self.message = self.socket.recv_string()
        if "A=" in self.message and "B=" in self.message:
            self.A = float(self.message.split("A")[1])
            self.B = float(self.message.split("B")[1])
        
    def start(self):
        self.servo1.source_delay = 0.01
        self.servo1.source = self.sin_values()

    def sin_values(self):
        angles = (2 * pi * i / self.period for i in range(self.period))
        for a in cycle(angles):
            yield self.A*sin(a)+self.B


##### End #####
servo1 = Servo1()
servo1.start()

while True:
    servo1.update()

