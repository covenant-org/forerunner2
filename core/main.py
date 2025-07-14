import time
import zmq

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://0.0.0.0:5551")

while True:
    print("waiting message")
    #  Wait for next request from client
    message = socket.recv()
    print("Received request: %s" % message)

    #  Do some 'work'
    # time.sleep(1)

    #  Send reply back to client
    # socket.send(b"World")
