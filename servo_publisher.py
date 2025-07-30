import zmq
import time

context = zmq.Context()
socket = context.socket(zmq.PUB)  # Use PUSH to send messages
socket.bind("tcp://*:7777")

a = 0
b = 0

while True:
    messA = float(input("Give A value: "))
    messB = float(input("Give b value: "))
    print("Sent Packet")
    socket.send_pyobj((messA,messB))  # Send the message
