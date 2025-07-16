#ifndef SUBSCRIBER_HPP
#define SUBSCRIBER_HPP

#include "message.hpp"
#include "utils.hpp"
#include <capnp/common.h>
#include <capnp/generated-header-support.h>
#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include <capnp/serialize.h>
#include <kj/common.h>
#include <kj/io.h>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <zmq.h>
#include <zmq.hpp>
#include <zmq_addon.hpp>

namespace Core {

template <typename T>
class Subscriber {
 private:
  std::string _topic;
  zmq::context_t _context;
  char _pub_add[30];
  std::function<void(IncomingMessage<T>)> _callback;
  std::thread* _listener_thread;

  // TODO: Should we make this in another thread so it won't block?
  bool query_registry(const std::string& registry_uri) {
    auto res = query_topic(_topic, registry_uri);
    if (!res.has_value()) return false;
    sprintf(_pub_add, "tcp://127.0.0.1:%d", res.value());
    return true;
  }

  void listen_to_new_messages() {
    zmq::context_t ctx(1);
    zmq::socket_t socket(ctx, zmq::socket_type::sub);
    socket.connect(_pub_add);
    socket.set(zmq::sockopt::subscribe, "");
    // TODO: Cleaner exit?
    while (true) {
      zmq::message_t msg;
      auto res = socket.recv(msg);
      if (res.has_value()) {
        _callback(IncomingMessage<T>((unsigned char*)msg.data(), res.value()));
      }
    }
  }

 public:
  Subscriber(const std::string& topic,
             std::function<void(IncomingMessage<T>)> callback)
      : _topic(std::move(topic)), _context(1), _callback(callback) {}

  void setup(const std::string& uri) {
    bool success = query_registry(uri);
    if (!success) {
      throw std::runtime_error("Error while querying registry");
    }
    _listener_thread = new std::thread(
        std::bind(&Subscriber<T>::listen_to_new_messages, this));
  }
};
}  // namespace Core

#endif
