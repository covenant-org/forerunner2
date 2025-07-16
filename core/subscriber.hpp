#ifndef SUBSCRIBER_HPP
#define SUBSCRIBER_HPP

#include "capnp_schemas/registry.capnp.h"
#include "message.hpp"
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

  bool query_registry(const std::string& registry_uri) {
    // Create capnp message
    ::capnp::MallocMessageBuilder msg;
    RegistryRequest::Builder req = msg.initRoot<RegistryRequest>();
    req.setType(RequestType::QUERY_NODE);
    req.setPath(_topic);

    // Backend to copy later to zmq (trying to not reallocate in the heap)
    std::string ptr(
        ::capnp::computeSerializedSizeInWords(msg) * sizeof(::capnp::word),
        '\0');
    auto array = ::kj::arrayPtr((::kj::byte*)ptr.data(), ptr.size());
    ::kj::ArrayOutputStream stream(array);
    ::capnp::writePackedMessage(stream, msg);

    // Copy message, connect and send
    zmq::message_t zreq(ptr.data(), stream.getArray().size());
    zmq::socket_t registry_sock(_context, ZMQ_REQ);
    registry_sock.connect(registry_uri);
    registry_sock.send(zreq, zmq::send_flags::none);

    // Preallocate memory and receive message
    // TODO: This is not the best way to do this
    zmq::message_t zres(::capnp::sizeInWords<RegistryResponse>() *
                        sizeof(::capnp::word));
    auto read = registry_sock.recv(zres);
    if (read.value_or(0) == 0) return false;

    // Deserialize the message
    auto res_array = ::kj::arrayPtr((::kj::byte*)zres.data(), read.value());
    ::kj::ArrayInputStream res_stream(res_array);
    ::capnp::PackedMessageReader res_reader(res_stream);
    RegistryResponse::Reader res = res_reader.getRoot<RegistryResponse>();
    if (res.which() == RegistryResponse::HOST) {
      auto host = res.getHost();
      sprintf(_pub_add, "tcp://%s:%d", host.getAddress().cStr(),
              host.getPort());
      printf("Topic '%s' found in %s\n", _topic.c_str(), _pub_add);
      return true;
    }
    if (res.which() == RegistryResponse::ERROR_MESSAGE) {
      printf("Querying '%s' got error %d: %s\n", _topic.c_str(), res.getCode(),
             res.getErrorMessage().cStr());
    }
    return false;
  }

  void listen_to_new_messages() {
    printf("help thread\n");
    zmq::context_t ctx(1);
    zmq::socket_t socket(ctx, zmq::socket_type::sub);
    socket.connect(_pub_add);
    socket.set(zmq::sockopt::subscribe, "");
    auto size = ::capnp::sizeInWords<T>() * sizeof(::capnp::word);
    // TODO: Cleaner exit?
    while (true) {
      zmq::message_t msg(size);
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
