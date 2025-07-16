#ifndef ACTION_HPP
#define ACTION_HPP

#include "capnp_schemas/registry.capnp.h"
#include "message.hpp"
#include "utils.hpp"
#include <capnp/common.h>
#include <capnp/generated-header-support.h>
#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include <capnp/serialize.h>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <kj/common.h>
#include <kj/io.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <zmq.hpp>
#include <zmq_addon.hpp>

namespace Core {

using RequestID = std::string;

template <typename T, typename K>
class ActionServer {
 private:
  std::string _topic;
  std::uint32_t _port;
  zmq::context_t _ctx;
  zmq::socket_t _router;
  zmq::socket_t _dealer;
  std::function<void(IncomingMessage<T>, typename K::Builder&)> _callback;
  uint8_t _num_workers;
  std::vector<std::thread*> _workers;
  std::thread* _main_thread;

  bool notify_registry(const std::string& registry_uri) {
    auto port = register_topic(_topic, registry_uri);
    if (!port.has_value()) return false;
    _port = port.value();
  }

  void worker(zmq::context_t ctx) {
    zmq::socket_t socket(ctx, zmq::socket_type::dealer);
    socket.connect("inproc://backend");
    try {
      while (true) {
        zmq::message_t identity;
        zmq::message_t msg;
        zmq::recv_result_t res = socket.recv(identity);
        if (res.value_or(0) == 0) continue;
        res = socket.recv(msg);
        if (res.value_or(0) == 0) continue;
        ::capnp::MallocMessageBuilder builder;
        _callback(IncomingMessage<T>((unsigned char*)msg.data(), msg.size()),
                  builder.getRoot<K>());
        kj::VectorOutputStream buffer;
        ::capnp::writePackedMessage(buffer, builder);
        auto serialized = buffer.getArray();
        zmq::message_t response(serialized.begin(), serialized.end());
        socket.send(identity, zmq::send_flags::sndmore);
        socket.send(response, zmq::send_flags::none);
      }
    } catch (std::exception) {
    }
  }

 public:
  ActionServer(
      const std::string& topic,
      std::function<void(IncomingMessage<T>, typename K::Builder&)> callback,
      uint8_t num_workers)
      : _topic(std::move(topic)),
        _port(0),
        _ctx(1),
        _router(_ctx, zmq::socket_type::router),
        _dealer(_ctx, zmq::socket_type::dealer),
        _callback(callback),
        _num_workers(num_workers) {}

  void setup_proxy() {
    char addr[20];
    sprintf(addr, "tcp://0.0.0.0:%d", _port);
    _router.bind(addr);
    _dealer.bind("inproc://backend");
    for (int i = 0; i < _num_workers; i++) {
      _workers.emplace_back(new std::thread(
          std::bind(&ActionServer::worker, this, std::placeholders::_1), _ctx));
    }
    zmq::proxy(_router, _dealer);
    for (int i = 0; i < _workers.size(); i++) {
      delete _workers[i];
    }
  }

  void setup(const std::string& uri) {
    auto success = this->notify_registry(uri);
    if (!success) {
      throw std::runtime_error("Error while creating action server");
    }
    _main_thread = new std::thread(std::bind(&ActionServer::setup_proxy, this));
  }
};

}  // namespace Core

#endif
