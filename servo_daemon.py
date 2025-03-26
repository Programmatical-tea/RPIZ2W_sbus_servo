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
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect("tcp://127.0.0.1:7777")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all messages
        self.latest_packet = (0,0)

    def update(self):
        while True:
            try:
                self.latest_packet = self.socket.recv_pyobj(flags=zmq.NOBLOCK)
            except zmq.Again:
                break
        self.A = self.latest_packet[0]
        self.B = self.latest_packet[1]
        print(f"updated value to {self.A}, {self.B}")
        
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

