# Need to install pyzmq using pip3 install pyzmq

import zmq
import time

# Set Pin Factory to pigpio https://gpiozero.readthedocs.io/en/stable/api_pins.html#changing-the-pin-factory
from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import Device, AngularServo
Device.pin_factory = PiGPIOFactory()


# Starting zmq Context, to read SBUS data from the Service Daemon
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://localhost:5555")
socket.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all messages
print("Starting SBUS subscription...")
time.sleep(0.5)


# Interpret the RF 'channel' list input
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
    return channels
    
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

# Move Servo continuously, Only updating the Amplitude.
def setServoState(active, amplitude):
    pass

while True:
    latest_packet = None

    while True:
        try:
            latest_packet = socket.recv_pyobj(flags=zmq.NOBLOCK)
        except zmq.Again:
            break
    
    if latest_packet is not None:
        channels, frame_lost, failsafe = parsePacket(latest_packet)
    
        if failsafe:
            print("Transmitter Connection LOST - Failsafe Activated!")
        elif frame_lost:
            print("Transmitter Connection unstable - Frame Lost detected!")
        else:
            print("Transmitter Connected")
        

        ### Main Loop Function Start ###


        channels_2_dir(channels)
        #print(f"Channels: {channels}")



        ### Main Loop Function End ###
    
    else:
        print("Turn the Transmitter on and check the Receiver connection!")

    time.sleep(0.2)

