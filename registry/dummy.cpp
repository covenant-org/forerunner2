#include <cstdio>
#include <iostream>
#include <zmq.hpp>

int main() {
  zmq::context_t ctx;
  zmq::socket_t sock(ctx, zmq::socket_type::req);
  sock.connect("tcp://127.0.0.1:4020");
  std::cout << "Connected" << std::endl;
  sock.send(zmq::str_buffer("hola\0"), zmq::send_flags::none);
  zmq::message_t msg(10);
  sock.recv(msg);
  printf("Received %s", (char*)msg.data());
  sock.close();
  return 0;
}
