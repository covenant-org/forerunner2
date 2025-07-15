#include "capnp_schemas/registry.capnp.h"
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

  bool notify_registry(const std::string& registry_uri) {
    ::capnp::MallocMessageBuilder msg;
    RegistryRequest::Builder req = msg.initRoot<RegistryRequest>();
    req.setType(RequestType::ADD_NODE);
    req.setPath(_topic);
    std::string ptr(
        ::capnp::computeSerializedSizeInWords(msg) * sizeof(::capnp::word),
        '\0');
    auto array = ::kj::arrayPtr((::kj::byte*)ptr.data(), ptr.size());
    ::kj::ArrayOutputStream stream(array);
    ::capnp::writePackedMessage(stream, msg);
    zmq::message_t zreq(ptr.data(), stream.getArray().size());
    zmq::socket_t registry_sock(_context, ZMQ_REQ);
    registry_sock.connect(registry_uri);
    registry_sock.send(zreq, zmq::send_flags::none);

    zmq::message_t zres(::capnp::sizeInWords<RegistryResponse>() *
                        sizeof(::capnp::word));
    auto status = registry_sock.recv(zres);
    if (status.value_or(0) == 0) {
      return false;
    }
    auto res_array = ::kj::arrayPtr((::kj::byte*)zres.data(), status.value());
    ::kj::ArrayInputStream res_stream(res_array);
    ::capnp::PackedMessageReader res_reader(res_stream);
    RegistryResponse::Reader res = res_reader.getRoot<RegistryResponse>();
    if (res.which() == RegistryResponse::HOST) {
      _port = res.getHost().getPort();
      printf("Publisher '%s' got the port %d\n", _topic.c_str(), _port);
      return true;
    }
    if (res.which() == RegistryResponse::ERROR_MESSAGE) {
      printf("Publisher '%s' got error %d: %s\n", _topic.c_str(), res.getCode(),
             res.getErrorMessage().cStr());
      return false;
    }
    return false;
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
