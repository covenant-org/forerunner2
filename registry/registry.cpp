#include "registry.hpp"
#include <iostream>

namespace Core {
Registry::Registry(int16_t port) : port(port) {}

void Registry::run() {
  zmq::socket_t sock(ctx, zmq::socket_type::router);
  char bind_dir[20];
  sprintf(bind_dir, "tcp://*:%d", port);
  sock.bind(std::string(bind_dir));
  const size_t size = 1024;
  void *ptr = malloc(size);
  zmq::mutable_buffer mbuf(ptr, size);
  while (true){
    std::optional<zmq::recv_buffer_size> res = sock.recv(mbuf, zmq::recv_flags::none);
    if (res.has_value()){
        printf("%ld: ", res.value().size);
        printf("%s\n", (char *)ptr);
    }
  }
}

}  // namespace Core

int main(int argc, char** argv) {
  Core::Registry registry(4020);
  registry.run();
  return 0;
}