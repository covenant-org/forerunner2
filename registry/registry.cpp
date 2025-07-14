#include "registry.hpp"
#include <cstdio>
#include <optional>
#include <thread>
#include <vector>
#include <zmq.h>
#include <zmq.hpp>

namespace Core {
Registry::Registry(RegistryConfiguration config)
    : config(config), ctx(1), router(ctx, ZMQ_ROUTER) {}

void Registry::handle_request() {}

void Registry::run() {
  char bind_dir[20];
  sprintf(bind_dir, "tcp://*:%d", config.port);
  router.bind(std::string(bind_dir));
  printf("Listening to %s\n", bind_dir);
  zmq::message_t identity(10);
  auto res = router.recv(identity);
  printf("Received %ld identity %d\n", res.value_or(0),
         *(uint *)identity.data());
  zmq::message_t empty(1);
  res = router.recv(empty);
  printf("Received %ld empty %d\n", res.value_or(0), *(uint *)empty.data());
  zmq::message_t data(10);
  res = router.recv(data);
  if (res.has_value()) {
    unsigned received_data = res.value();
    printf("Received %d data %s\n", received_data, (char *)data.data());
  }
}

}  // namespace Core

int main(int argc, char **argv) {
  Core::Registry registry({.port = 4020});
  printf("help\n");
  registry.run();
  return 0;
}
