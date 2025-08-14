#ifndef CORE_UTILS_HPP
#define CORE_UTILS_HPP

#include "capnp_schemas/registry.capnp.h"
#include "logger.hpp"
#include "message.hpp"
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <kj/common.h>
#include <optional>
#include <stdexcept>
#include <string>
#include <zmq.hpp>

namespace Core {

inline std::string find_root(const std::string& filename = ".root",
                             int max_levels = 10) {
  std::filesystem::path current = std::filesystem::current_path();
  for (int i = 0; i <= max_levels; ++i) {
    for (const auto& entry : std::filesystem::directory_iterator(current)) {
      if (entry.is_regular_file() && entry.path().filename() == filename) {
        return entry.path().parent_path().string();
      }
    }
    if (current.has_parent_path()) {
      current = current.parent_path();
    } else {
      break;
    }
  }
  Logger(LogLevel::ERROR, "application.log", "Utils").error("Root not found");
  return "";
}

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

template <unsigned int N>
class StabilityMonitor {
 private:
  float _values[N];
  unsigned int _length;

 public:
  StabilityMonitor() {
    _length = 0;
    for (unsigned int i = 0; i < N; i++) {
      _values[i] = 0;
    }
  }

  void add_value(float value) {
    for (unsigned int i = N - 1; i > 0; i--) {
      _values[i] = _values[i - 1];
    }
    _values[0] = value;
    _length++;
    if (_length > N) {
      _length = N;
    }
  }

  bool is_stable(float threshold = 0.1) {
    if (_length < N / 3) return false;

    auto stdDev = calculate_std_dev();
    return stdDev < threshold;  // Low std dev = stable
  }

  float calculate_std_dev() {
    float mean = 0;
    for (unsigned int i; i < _length; i++) {
      mean += _values[i];
    }
    mean /= _length;
    float variance = 0;
    for (unsigned int i; i < _length; i++) {
      variance += std::pow(_values[i] - mean, 2);
    }
    variance /= _length;
    return std::sqrt(variance);
  }
};

}  // namespace Core

#endif
