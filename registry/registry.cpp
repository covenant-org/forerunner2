#include "capnp_schemas/registry.capnp.h"
#include "registry.hpp"
#include <capnp/common.h>
#include <capnp/generated-header-support.h>
#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include <capnp/serialize.h>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <kj/common.h>
#include <kj/io.h>
#include <optional>
#include <stdexcept>
#include <string>
#include <zmq.h>
#include <zmq.hpp>

namespace Core {
Registry::Registry(RegistryConfiguration config)
    : _config(config),
      _ctx(config.threads),
      _router(_ctx, ZMQ_ROUTER),
      _last_free_port(config.port) {
  this->_logger.set_classname("registry");
}

zmq::message_t Registry::message_from_builder(
    ::capnp::MallocMessageBuilder &message) {
  std::string ptr(
      ::capnp::computeSerializedSizeInWords(message) * sizeof(capnp::word),
      '\0');
  auto array = ::kj::arrayPtr((kj::byte *)ptr.data(), ptr.size());
  ::kj::ArrayOutputStream output(array);
  ::capnp::writePackedMessage(output, message);
  auto packed_size = output.getArray().size();

  return zmq::message_t(ptr.data(), packed_size);
}

void Registry::respond_event(RouterEvent &event, zmq::message_t msg) {
  _router.send(event.identity, zmq::send_flags::sndmore);
  _router.send(zmq::message_t(), zmq::send_flags::sndmore);
  _router.send(msg, zmq::send_flags::none);
}

void Registry::notify_waiters(std::string path) {
  try {
    auto endpoint = _topic_to_endpoint[path];
    for (auto id : _topic_to_waiters[path]) {
      RouterEvent wait_event{
          .identity = zmq::message_t(id.data(), id.size()),
          .data = zmq::message_t(),
      };
      ::capnp::MallocMessageBuilder message;
      RegistryResponse::Builder res = message.initRoot<RegistryResponse>();
      res.setCode(200);
      auto host = res.initHost();
      host.setAddress(endpoint.host);
      host.setPort(endpoint.port);
      respond_event(wait_event, message_from_builder(message));
    }
  } catch (std::out_of_range) {
  }
}

void Registry::handle_request(RouterEvent event) {
  auto array = ::kj::arrayPtr((uint8_t *)event.data.data(), event.data.size());
  auto stream = ::kj::ArrayInputStream(array);
  ::capnp::PackedMessageReader message(stream);
  RegistryRequest::Reader request = message.getRoot<RegistryRequest>();

  if (request.getType() == RequestType::ADD_NODE) {
    ::capnp::MallocMessageBuilder message;
    RegistryResponse::Builder res = message.initRoot<RegistryResponse>();
    auto free_port = get_free_port();
    if (!free_port.has_value()) {
      res.setCode(408);
      res.setErrorMessage("Could not find a free port within range");
      respond_event(event, message_from_builder(message));
      return;
    }
    std::string path = request.getPath();
    Endpoint endpoint{
        .host = "127.0.0.1",
        .port = free_port.value(),
    };
    _topic_to_endpoint.insert_or_assign(path, endpoint);
    this->_logger.info("New topic: %s at %d", path.c_str(), endpoint.port);
    res.setCode(201);
    auto host = res.initHost();
    host.setAddress(endpoint.host);
    host.setPort(endpoint.port);
    respond_event(event, message_from_builder(message));
    notify_waiters(path);
    return;
  }

  if (request.getType() == RequestType::QUERY_NODE) {
    auto path = request.getPath();
    this->_logger.debug("querying topic: %s", request.getPath().cStr());
    try {
      Endpoint node = _topic_to_endpoint.at(path);
      this->_logger.debug("querying topic [%s] found at %d",
                          request.getPath().cStr(), node.port);
      ::capnp::MallocMessageBuilder message;
      RegistryResponse::Builder res = message.initRoot<RegistryResponse>();
      res.setCode(200);
      auto host = res.initHost();
      host.setAddress(node.host);
      host.setPort(node.port);
      respond_event(event, message_from_builder(message));
    } catch (std::out_of_range) {
      this->_logger.debug("querying topic not found, pending to notify");
      _topic_to_waiters[path].emplace_back((char *)event.identity.data(),
                                           event.identity.size());
    }
    return;
  }

  // NOTE: Not so sure about this one
  if (request.getType() == RequestType::ADD_HOST) {
    ::capnp::MallocMessageBuilder message;
    RegistryResponse::Builder res = message.initRoot<RegistryResponse>();
    auto obj = request.getAddHost();
    auto path = request.getPath();
    Endpoint endpoint{
        .host = obj.getAddress(),
        .port = obj.getPort(),
    };
    _topic_to_endpoint.insert_or_assign(request.getPath(), endpoint);
    this->_logger.info("New Host: %s at %d", path.cStr(), endpoint.port);
    res.setCode(201);
    auto host = res.initHost();
    host.setAddress(endpoint.host);
    host.setPort(endpoint.port);
    respond_event(event, message_from_builder(message));
    notify_waiters(path);
  }
}

std::optional<uint32_t> Registry::get_free_port() {
  zmq::context_t tmp_ctx(1);
  zmq::socket_t tmp(tmp_ctx, ZMQ_REQ);
  for (int i = 1; i < MAX_PORT_SEARCHES; i++) {
    char bind_dir[20];
    sprintf(bind_dir, "tcp://0.0.0.0:%d", _last_free_port + i);
    try {
      tmp.bind(bind_dir);
      tmp.unbind(bind_dir);
      _last_free_port += i;
      return _last_free_port;
    } catch (zmq::error_t error) {
      this->_logger.error("get_free_port failed due to: %s", error.what());
    }
  }
  return std::nullopt;
}

std::optional<RouterEvent> Registry::wait_for_message(zmq::socket_t &socket) {
  // Router gets the identity first
  zmq::message_t identity(5);
  auto res = _router.recv(identity);
  if (res.value_or(0) == 0) {
    this->_logger.error("Bad message while waiting for id");
    return std::nullopt;
  }

  zmq::message_t empty;
  res = _router.recv(empty);
  if (res.value_or(0) != 0) {
    this->_logger.error("Bad message while waiting for package separator");
    return std::nullopt;
  }

  zmq::message_t data(1024);
  res = _router.recv(data);
  if (res.has_value()) {
    return std::optional<RouterEvent>(
        RouterEvent{.identity = std::move(identity), .data = std::move(data)});
  }
  return std::nullopt;
}

void Registry::run() {
  char bind_dir[20];
  sprintf(bind_dir, "tcp://*:%d", _config.port);
  _router.bind(std::string(bind_dir));
  this->_logger.info("Listening to %s", bind_dir);
  while (true) {
    auto event = wait_for_message(_router);
    if (!event.has_value()) continue;
    handle_request(std::move(event.value()));
  }
}

}  // namespace Core

int main(int argc, char **argv) {
  Core::Registry registry({.port = 4020, .threads = 5});
  registry.run();
  return 0;
}
