import time
import zmq
import capnp

capnp.remove_import_hook()
mavlink_messages = capnp.load('../messages/schemas/mavlink.capnp')

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://127.0.0.1:5551")
socket.setsockopt_string(zmq.SUBSCRIBE, "")

while True:
    data = socket.recv()
    msg = mavlink_messages.Telemetry.from_bytes_packed(data)
    print(msg)
