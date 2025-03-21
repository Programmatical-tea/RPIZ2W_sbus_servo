import serial
import struct

# SBUS Constants
SBUS_FRAME_LENGTH = 25
SBUS_BAUDRATE = 100000
SBUS_HEADER = 0x0F
SBUS_FOOTER = 0x00

# Open the UART port
ser = serial.Serial(
    port="/dev/ttyAMA0",  # Raspberry Pi hardware UART
    baudrate=SBUS_BAUDRATE,
    parity=serial.PARITY_EVEN,
    stopbits=serial.STOPBITS_TWO,
    bytesize=serial.EIGHTBITS,
    timeout=0.1  # Short timeout to avoid blocking
)

def parse_sbus_frame(frame):
    """ Parses a 25-byte SBUS frame and extracts channel data """
    if len(frame) != SBUS_FRAME_LENGTH:
        return None  # Invalid frame size

    if frame[0] != SBUS_HEADER or frame[-1] not in [SBUS_FOOTER, 0x04]:  
        return None  # Invalid header/footer

    # Decode 16 channels (each channel is 11-bit packed in 22 bytes)
    channels = [0] * 16
    bitstream = int.from_bytes(frame[1:23], byteorder="little")  # Convert bytes to a bitstream
    
    for i in range(16):
        channels[i] = (bitstream >> (i * 11)) & 0x7FF  # Extract each 11-bit channel

    # Read Flags
    failsafe = (frame[23] & 0x08) >> 3  # Failsafe status
    frame_lost = (frame[23] & 0x04) >> 2  # Frame lost status

    return channels, failsafe, frame_lost

def read_sbus():
    """ Continuously reads SBUS frames and decodes them """
    i = 0
    while True:
        i += 1
        print(i)
        if ser.in_waiting >= SBUS_FRAME_LENGTH:
            raw_data = ser.read(SBUS_FRAME_LENGTH)  # Read a full SBUS packet
            parsed = parse_sbus_frame(raw_data)
            if parsed:
                channels, failsafe, frame_lost = parsed
                print(f"Channels: {channels}")
                print(f"Failsafe: {failsafe}, Frame Lost: {frame_lost}\n")

try:
    print("Listening for SBUS data...")
    read_sbus()
except KeyboardInterrupt:
    print("Stopping...")
finally:
    ser.close()
