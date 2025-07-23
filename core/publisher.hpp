#ifndef PUBLISHER_HPP
#define PUBLISHER_HPP

#include "message.hpp"
#include "utils.hpp"
#include <capnp/common.h>
#include <capnp/generated-header-support.h>
#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include <capnp/serialize.h>
#include <kj/common.h>
#include <kj/io.h>
#include <stdexcept>
#include <string>
#include <zmq.hpp>
#include <zmq_addon.hpp>

namespace Core {
template <typename T>
class Publisher : public ISender {
 private:
  std::string _topic;
  uint32_t _port;
  zmq::context_t _context;
  zmq::socket_t _socket;

  bool notify_registry(const std::string& registry_uri) {
    auto res = register_topic(_topic, registry_uri);
    if (!res.has_value()) return false;
    _port = res.value();
    return true;
  }

 public:
  Publisher(const std::string& topic)
      : _topic(std::move(topic)),
        _port(0),
        _context(1),
        _socket(_context, zmq::socket_type::pub) {}

  void setup(const std::string& uri) {
    auto success = this->notify_registry(uri);
    if (!success) {
      throw std::runtime_error("error with registry");
    }

    char buffer[20];
    sprintf(buffer, "tcp://0.0.0.0:%d", _port);
    _socket.bind(buffer);
  }

  OutgoingMessage<T> new_msg() { return OutgoingMessage<T>(this); }

  uint32_t publish(::capnp::MallocMessageBuilder& builder) {
    kj::VectorOutputStream buffer;
    ::capnp::writePackedMessage(buffer, builder);
    auto serialized = buffer.getArray();

    zmq::message_t zmq_message(serialized.size());
    memcpy(zmq_message.data(), serialized.begin(), serialized.size());
    this->_socket.send(zmq_message, zmq::send_flags::none);

    return 0;
  }
};
};  // namespace Core

#endif  // PUBLISHER_HPP
