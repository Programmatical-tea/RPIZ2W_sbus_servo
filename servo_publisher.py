import zmq
import time

context = zmq.Context()
socket = context.socket(zmq.PUSH)  # Use PUSH to send messages
socket.bind("tcp://127.0.0.1:7777")  # Bind to a port

a = 0
b = 0

while True:
    messA = float(input("Give A value: "))
    messB = float(input("Give b value: "))
    message = f"A{messA}A B{messB}B"
    socket.send_string(message)  # Send the message
