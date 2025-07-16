#ifndef CORE_UTILS_HPP
#define CORE_UTILS_HPP

#include "capnp_schemas/registry.capnp.h"
#include "message.hpp"
#include <cstdint>
#include <iostream>
#include <kj/common.h>
#include <optional>
#include <stdexcept>
#include <string>
#include <zmq.hpp>

namespace Core {

inline IncomingMessage<RegistryResponse> _make_request_to_registry(
    const std::string& topic, RequestType type, const std::string& uri) {
  // Create capnp message
  ::capnp::MallocMessageBuilder msg;
  RegistryRequest::Builder req = msg.initRoot<RegistryRequest>();
  req.setType(type);
  req.setPath(topic);

  // Backend to copy later to zmq (trying to not reallocate in the heap)
  std::string ptr(
      ::capnp::computeSerializedSizeInWords(msg) * sizeof(::capnp::word), '\0');
  auto array = ::kj::arrayPtr((::kj::byte*)ptr.data(), ptr.size());
  ::kj::ArrayOutputStream stream(array);
  ::capnp::writePackedMessage(stream, msg);

  // Copy message, connect and send
  zmq::context_t ctx(1);
  zmq::message_t zreq(ptr.data(), stream.getArray().size());
  zmq::socket_t registry_sock(ctx, ZMQ_REQ);
  registry_sock.connect(uri);
  registry_sock.send(zreq, zmq::send_flags::none);

  // Preallocate memory and receive message
  // TODO: This is not the best way to do this
  zmq::message_t zres(::capnp::sizeInWords<RegistryResponse>() *
                      sizeof(::capnp::word));
  auto read = registry_sock.recv(zres);
  if (read.value_or(0) == 0)
    throw std::runtime_error("Empty response from registry");

  // Deserialize the message
  return IncomingMessage<RegistryResponse>((unsigned char*)zres.data(),
                                           read.value());
}

inline std::optional<uint32_t> register_topic(const std::string& topic,
                                              const std::string& uri) {
  try {
    auto msg = _make_request_to_registry(topic, RequestType::ADD_NODE, uri);
    if (msg.content.which() == RegistryResponse::ERROR_MESSAGE) {
      std::cerr << "Error while registering topic: "
                << msg.content.getErrorMessage().cStr() << std::endl;
      return std::nullopt;
    }
    return msg.content.getHost().getPort();
  } catch (std::runtime_error& error) {
    std::cerr << "Error while registering topic: " << error.what() << std::endl;
    return std::nullopt;
  }
}

inline std::optional<uint32_t> query_topic(const std::string& topic,
                                           const std::string& uri) {
  try {
    auto msg = _make_request_to_registry(topic, RequestType::QUERY_NODE, uri);
    if (msg.content.which() == RegistryResponse::ERROR_MESSAGE) {
      std::cerr << "Error while querying topic: "
                << msg.content.getErrorMessage().cStr() << std::endl;
      return std::nullopt;
    }
    return msg.content.getHost().getPort();
  } catch (std::runtime_error& error) {
    std::cerr << "Error while querying topic: " << error.what() << std::endl;
    return std::nullopt;
  }
}

}  // namespace Core

#endif
