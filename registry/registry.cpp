#include "registry.hpp"
#include <csignal>
#include <cstdio>
#include <iostream>
#include <optional>
#include <zmq.h>
#include <zmq.hpp>

namespace Core {
Registry::Registry(RegistryConfiguration config)
    : config(config), ctx(config.threads), router(ctx, ZMQ_ROUTER) {}

void Registry::handle_request(RouterEvent event) {
  router.send(event.identity, zmq::send_flags::sndmore);
  router.send(zmq::message_t(), zmq::send_flags::sndmore);
  router.send(zmq::str_buffer("World\0"), zmq::send_flags::none);
}

std::optional<RouterEvent> Registry::wait_for_message(zmq::socket_t &socket) {
  // Router gets the identity first
  zmq::message_t identity(5);
  auto res = router.recv(identity);
  if (res.value_or(0) == 0) {
    return std::nullopt;
    std::cerr << "Bad message while waiting for id" << std::endl;
  }

  zmq::message_t empty;
  res = router.recv(empty);
  if (res.value_or(0) != 0) {
    std::cerr << "Bad message while waiting for package separator" << std::endl;
    return std::nullopt;
  }

  zmq::message_t data(1024);
  res = router.recv(data);
  if (res.has_value()) {
    printf("Received data %s\n", (char *)data.data());
    return std::optional<RouterEvent>(
        RouterEvent{.identity = std::move(identity), .data = std::move(data)});
  }
  return std::nullopt;
}

void Registry::run() {
  char bind_dir[20];
  sprintf(bind_dir, "tcp://*:%d", config.port);
  router.bind(std::string(bind_dir));
  printf("Listening to %s\n", bind_dir);

  while (true) {
    auto event = wait_for_message(router);
    if (!event.has_value()) continue;
    handle_request(std::move(event.value()));
  }
}

}  // namespace Core

int main(int argc, char **argv) {
  Core::Registry registry({.port = 4020, .threads = 5});
  printf("help\n");
  registry.run();
  return 0;
}
