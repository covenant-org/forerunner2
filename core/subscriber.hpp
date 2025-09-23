#ifndef SUBSCRIBER_HPP
#define SUBSCRIBER_HPP

#include "message.hpp"
#include "utils.hpp"
#include <atomic>
#include <capnp/common.h>
#include <capnp/generated-header-support.h>
#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include <capnp/serialize.h>
#include <cstdint>
#include <kj/common.h>
#include <kj/exception.h>
#include <kj/io.h>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <zmq.h>
#include <zmq.hpp>
#include <zmq_addon.hpp>

namespace Core {

class ISubscriber {
 public:
  virtual ~ISubscriber() = default;
  virtual uint32_t reset_connection() = 0;
  virtual void stop() = 0;
};

template <typename T>
class Subscriber : ISubscriber {
 private:
  std::string _topic;
  zmq::context_t _context;
  char _pub_add[30];
  std::function<void(IncomingMessage<T>)> _callback;
  std::thread* _listener_thread;
  std::string uri;
  std::atomic_bool _exit;
  Logger _logger;

  // TODO: Should we make this in another thread so it won't block?
  bool query_registry(const std::string& registry_uri) {
    auto res = query_topic(_topic, registry_uri);
    if (!res.has_value()) return false;
    auto value = res.value();
    sprintf(_pub_add, "tcp://%s:%d", value.first.c_str(), value.second);
    return true;
  }

  void listen_to_new_messages() {
    bool success = query_registry(uri);
    if (!success) {
      throw std::runtime_error("Error while querying registry");
    }
    zmq::context_t ctx(1);
    zmq::socket_t socket(ctx, zmq::socket_type::sub);
    socket.connect(_pub_add);
    socket.set(zmq::sockopt::subscribe, "");
    // TODO: Cleaner exit?
    while (!this->_exit) {
      zmq::message_t msg;
      auto res = socket.recv(msg);
      if (res.has_value()) {
        try {
          _callback(
              IncomingMessage<T>((unsigned char*)msg.data(), res.value()));
        } catch (kj::Exception ex) {
          _logger.error("Error while deserializing: %s", ex.getDescription());
        }
      }
    }
    socket.close();
    ctx.shutdown();
    ctx.close();
  }

 public:
  Subscriber(const std::string& topic,
             std::function<void(IncomingMessage<T>)> callback,
             std::string vertex_name = "")
      : _topic(std::move(topic)),
        _context(1),
        _callback(callback),
        _exit(false) {
    this->_logger.set_classname(vertex_name + "-Subscriber-" + this->_topic);
  }

  uint32_t reset_connection() {
    this->_logger.debug("Requested reconnection");
    this->stop();
    this->_logger.debug("Stopped current thread");
    this->setup(this->uri);
    this->_logger.debug("Ready Suybscriber");
    return 0;
  }

  void setup(const std::string& uri) {
    this->uri = uri;
    _listener_thread = new std::thread(
        std::bind(&Subscriber<T>::listen_to_new_messages, this));
  }

  void stop() {
    this->_exit = true;
    if (this->_listener_thread->joinable()) this->_listener_thread->join();
  }
};
}  // namespace Core

#endif
