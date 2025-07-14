#include <iostream>
#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include <kj/io.h>
#include <string>
#include <thread>
#include <zmq.hpp>
#include <zmq_addon.hpp>

#ifndef PUBLISHER_HPP
#define PUBLISHER_HPP

namespace Core {
template <typename T>
class Publisher {
 private:
  std::string _topic;
  uint32_t _port;
  zmq::context_t _context;
  zmq::socket_t _socket;

  bool notify_registry(const std::string&) {
    // TODO: make connection with registry
    return true;
  }

 public:
  Publisher(const std::string& topic, uint32_t port)
      : _topic(std::move(topic)),
        _port(port),
        _socket(_context, zmq::socket_type::pub) {}

  void setup(const std::string& uri) {
    auto success = this->notify_registry(uri);
    if (!success) {
      throw "error with registry";
    }

    char buffer[20];
    sprintf(buffer, "tcp://0.0.0.0:%d", _port);
    _socket.bind(buffer);
  }

  uint32_t publish(::capnp::MallocMessageBuilder& builder) {
    kj::VectorOutputStream buffer;
    ::capnp::writePackedMessage(buffer, builder);
    auto serialized = buffer.getArray();

    // TODO: check if we con avoid the copy
    zmq::message_t zmq_message(serialized.size());
    memcpy(zmq_message.data(), serialized.begin(), serialized.size());
    std::cout << "sent message" << std::endl;
    this->_socket.send(zmq_message, zmq::send_flags::none);

    return 0;
  }
};
};  // namespace Core

#endif  // PUBLISHER_HPP
