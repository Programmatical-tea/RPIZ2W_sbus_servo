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
import threading

SERVO_GPIO_PIN_1 = 12

def map_2_range(value, old_min=200, old_max=1800, new_min=-10, new_max=10):
    if value < old_min or value > old_max:
        print(f"Value = {value}")
        raise ValueError(f"Mapping Value is out of range, Value={value}")
    new_value = int((value-old_min) * (new_max-new_min) / (old_max-old_min) + new_min)
    return new_value

def channels_2_dir(old_channels):
    assert len(old_channels) == 18
    channels = list(map(map_2_range, old_channels[0:16]))
    LeftVert = channels[2]
    LeftHori = channels[3]
    RightVert = channels[1]
    RightHori = channels[0]
    LeftSwitch = channels[6]
    print(f"LeftVert={LeftVert}, LeftHori={LeftHori}")
    print(f"RightVert={RightVert}, RightHori={RightHori}")
    print(f"SwitchOn={LeftSwitch}")
    return LeftVert, RightHori, LeftSwitch
    
# Parse packet into coherent 'channel' list
def parsePacket(packet):
    channel = [-1] * 18
    channel[0] = (packet[2] << 8 & 0b0111_0000_0000) | packet[1]
    channel[1] = (packet[3] << 5 & 0b0111_1110_0000) | (packet[2] >> 3)
    channel[2] = (packet[5] << 10 & 0b0100_0000_0000) | (packet[4] << 2) | (packet[3] >> 6)
    channel[3] = (packet[6] << 7 & 0b0111_1000_0000) | (packet[5] >> 1)
    channel[4] = (packet[7] << 4 & 0b0111_1111_0000) | (packet[6] >> 4)
    channel[5] = (packet[9] << 9 & 0b0110_0000_0000) | (packet[8] << 1) | (packet[7] >> 7)
    channel[6] = (packet[10] << 6 & 0b0111_1100_0000) | (packet[9] >> 2)
    channel[7] = (packet[11] << 3) | (packet[10] >> 5)
    channel[8] = (packet[13] << 8 & 0b0111_0000_0000) | packet[12]
    channel[9] = (packet[14] << 5 & 0b0111_1110_0000) | (packet[13] >> 3)
    channel[10] = (packet[16] << 10 & 0b0100_0000_0000) | (packet[15] << 2) | (packet[14] >> 6)
    channel[11] = (packet[17] << 7 & 0b0111_1000_0000) | (packet[16] >> 1)
    channel[12] = (packet[18] << 4 & 0b0111_1111_0000) | (packet[17] >> 4)
    channel[13] = (packet[20] << 9 & 0b0110_0000_0000) | (packet[19] << 1) | (packet[18] >> 7)
    channel[14] = (packet[21] << 6 & 0b0111_1100_0000) | (packet[20] >> 2)
    channel[15] = (packet[22] << 3) | (packet[21] >> 5)
    channel[16] = packet[23] & 0b00000001
    channel[17] = packet[23] & 0b00000010

    frame_lost = bool(packet[23] & 0b00000100)
    failsafe = bool(packet[23] & 0b00001000)
    
    return channel, frame_lost, failsafe

class Servo1:

    def __init__(self):
        self.A = 0
        self.B = 0
        self.period = 360
        self.servo1 = AngularServo(SERVO_GPIO_PIN_1,min_angle=-40, max_angle=40, min_pulse_width=0.0009, max_pulse_width=0.0019)
        self.servo1.source_delay = 0.01
        self.servo1.source = self.sin_values()

        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect("tcp://localhost:7777")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all messages
        self.latest_packet = (0,0)

        self.rf_socket = self.context.socket(zmq.SUB)
        self.rf_socket.connect("tcp://localhost:5555")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")
        print("Starting SBUS subscription...")
        self.latest_rfpacket = None

        self.running = True
        self.thread = threading.Thread(target=self.update, daemon=True)

        self.channels = None
        

    def update(self):

        while self.running:
            while True:
                try:
                    self.latest_packet = self.socket.recv_pyobj(flags=zmq.NOBLOCK)
                except zmq.Again:
                    #print("This is Again")
                    break

            while True:
                try:
                    self.latest_rfpacket = self.rf_socket.recv_pyobj(flags=zmq.NOBLOCK)
                    print(self.latest_rfpacket)
                except zmq.Again:
                    #print("This is Again2")
                    break
            #print("new try")
            print(self.latest_packet) # this works
            print(self.latest_rfpacket) # Not this.

            if self.latest_rfpacket is not None:
                channels, frame_lost, failsafe = parsePacket(self.latest_rfpacket)
                print("Trasmit input")
                if failsafe:
                    print("Transmitter Connection LOST - Failsafe Activated!")
                elif frame_lost:
                    print("Transmitter Connection unstable - Frame Lost detected!")
                else:
                    print("Transmitter Connected")

                self.channels = channels_2_dir(channels)
                print(self.channels[2])
                if self.channels[2] > 0:
                    if self.latest_packet is not None:
                        self.A = self.latest_packet[0]
                        self.B = self.latest_packet[1]
                        print(f"updated value to {self.A}, {self.B}")
                else:
                    self.A = self.channels[0]/10
                    self.B = self.channels[1]/10

    def start(self):
        self.thread.start()

    def stop(self):
        self.running = False
        self.thread.join()

    def sin_values(self):
        angles = (2 * pi * i / self.period for i in range(self.period))
        for a in cycle(angles):
            yield self.A*sin(a)+self.B




##### End #####
servo1 = Servo1()
servo1.start()

input("Stop?")

servo1.stop()
