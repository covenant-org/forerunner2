#ifndef ACTION_HPP
#define ACTION_HPP

#include "message.hpp"
#include "subscriber.hpp"
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
class ActionServer : public ISender {
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
    return true;
  }

  void worker() {
    zmq::socket_t socket(_ctx, zmq::socket_type::dealer);
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
        typename K::Builder root = builder.getRoot<K>();
        _callback(IncomingMessage<T>((unsigned char*)msg.data(), msg.size()),
                  root);
        kj::VectorOutputStream buffer;
        ::capnp::writePackedMessage(buffer, builder);
        auto serialized = buffer.getArray();
        zmq::message_t response(serialized.begin(), serialized.end());
        socket.send(identity, zmq::send_flags::sndmore);
        socket.send(response, zmq::send_flags::none);
      }
    } catch (const std::exception&) {
    }
  }

 public:
  ActionServer(
      const std::string& topic,
      std::function<void(IncomingMessage<T>, typename K::Builder&)> callback,
      uint8_t num_workers = 5)
      : _topic(std::move(topic)),
        _port(0),
        _ctx(1),
        _router(_ctx, zmq::socket_type::router),
        _dealer(_ctx, zmq::socket_type::dealer),
        _callback(callback),
        _num_workers(num_workers),
        _main_thread(nullptr) {}

  void setup_proxy() {
    char addr[20];
    sprintf(addr, "tcp://0.0.0.0:%d", _port);
    _router.bind(addr);
    _dealer.bind("inproc://backend");
    for (size_t i = 0; i < _num_workers; i++) {
      _workers.emplace_back(
          new std::thread(std::bind(&ActionServer::worker, this)));
    }
    zmq::proxy(_router, _dealer);
    for (size_t i = 0; i < _workers.size(); i++) {
      delete _workers[i];
    }
  }

  uint32_t publish(::capnp::MallocMessageBuilder& builder) {}

  uint32_t reset_connection(const std::string& uri) {
    char addr[20];
    sprintf(addr, "tcp://0.0.0.0:%d", _port);
    _router.unbind(addr);
    auto success = this->notify_registry(uri);
    if (!success) {
      throw std::runtime_error("Error while creating action server");
    }
    sprintf(addr, "tcp://0.0.0.0:%d", _port);
    _router.bind(addr);
  }

  void setup(const std::string& uri) {
    auto success = this->notify_registry(uri);
    if (!success) {
      throw std::runtime_error("Error while creating action server");
    }
    _main_thread = new std::thread(std::bind(&ActionServer::setup_proxy, this));
  }
};

template <typename K>
class ITransaction {
 public:
  virtual ~ITransaction() = default;
  virtual std::optional<IncomingMessage<K>> send(
      ::capnp::MallocMessageBuilder& builder) = 0;
};

template <typename T, typename K>
class ActionRequest {
 private:
  ::capnp::MallocMessageBuilder builder;
  ITransaction<K>* sender;

 public:
  typename T::Builder content;
  ~ActionRequest() {}
  ActionRequest(const ActionRequest<T, K>& a)
      : sender(a.sender), content(builder.getRoot<T>()) {
    builder.setRoot(a.content);
    content = builder.getRoot<T>();
  }
  ActionRequest(ITransaction<K>* sender)
      : builder(), sender(sender), content(builder.getRoot<T>()) {}
  std::optional<IncomingMessage<K>> send() { return sender->send(builder); }
};

template <typename T, typename K>
class ActionClient : public ITransaction<K>, public ISubscriber {
 private:
  std::string _topic;
  std::string _uri;
  uint32_t _port;
  zmq::context_t _context;
  zmq::socket_t _socket;

  bool query_registry(const std::string& registry_uri) {
    auto res = query_topic(_topic, registry_uri);
    if (!res.has_value()) return false;
    _port = res.value().second;
    return true;
  }

 public:
  ActionClient(const std::string& topic)
      : _topic(std::move(topic)),
        _port(0),
        _context(1),
        _socket(_context, zmq::socket_type::dealer) {}

  void setup(const std::string& uri) {
    this->_uri = uri;
    auto success = this->query_registry(uri);
    if (!success) {
      throw std::runtime_error("error with registry");
    }

    char buffer[30];
    sprintf(buffer, "tcp://127.0.0.1:%d", _port);
    _socket.connect(buffer);
  }

  ActionRequest<T, K> new_msg() { return ActionRequest<T, K>(this); }

  void stop() {}

  uint32_t reset_connection() {
    char buffer[30];
    sprintf(buffer, "tcp://127.0.0.1:%d", _port);
    _socket.disconnect(buffer);
    this->setup(this->_uri);
    return 0;
  }

  std::optional<IncomingMessage<K>> send(
      ::capnp::MallocMessageBuilder& builder) {
    kj::VectorOutputStream buffer;
    ::capnp::writePackedMessage(buffer, builder);
    auto serialized = buffer.getArray();

    zmq::message_t zmq_message(serialized.size());
    memcpy(zmq_message.data(), serialized.begin(), serialized.size());
    this->_socket.send(zmq_message, zmq::send_flags::none);

    zmq::message_t msg;
    zmq::recv_result_t res = this->_socket.recv(msg);
    if (res.value_or(0) == 0) return std::optional<IncomingMessage<K>>();
    return IncomingMessage<K>((unsigned char*)msg.data(), msg.size());
  }
};

}  // namespace Core

#endif
