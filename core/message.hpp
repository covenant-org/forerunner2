#ifndef MESSAGE_HPP
#define MESSAGE_HPP

#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include <capnp/serialize.h>
#include <kj/common.h>
#include <kj/io.h>
#include <zmq.hpp>

namespace Core {
class ISender {
 public:
  virtual ~ISender() = default;
  virtual uint32_t publish(::capnp::MallocMessageBuilder& builder) = 0;
};

template <typename T>
class OutgoingMessage {
 private:
  ::capnp::MallocMessageBuilder builder;
  ISender* sender;

 public:
  typename T::Builder content;
  OutgoingMessage(ISender* sender)
      : builder(), sender(sender), content(builder.getRoot<T>()) {}
  uint32_t publish() { return sender->publish(builder); }
};

template <typename T>
class IncomingMessage {
 private:
  // TODO: Evaluate if copying is necesary
  std::string buffer;
  ::kj::ArrayPtr<unsigned char> ptr;
  ::kj::ArrayInputStream array;

  ::capnp::PackedMessageReader reader;

 public:
  typename T::Reader content;
  IncomingMessage(const IncomingMessage<T>& a)
      : buffer(a.buffer.data(), a.buffer.size()),
        ptr((unsigned char*)buffer.data(), a.buffer.size()),
        array(ptr),
        reader(array),
        content(reader.getRoot<T>()) {}

  IncomingMessage(unsigned char* data, uint32_t size)
      : buffer((char*)data, size),
        ptr((unsigned char*)buffer.data(), size),
        array(ptr),
        reader(array),
        content(reader.getRoot<T>()) {}
};

}  // namespace Core

#endif
