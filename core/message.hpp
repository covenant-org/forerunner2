#ifndef MESSAGE_HPP
#define MESSAGE_HPP

#include <capnp/message.h>

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

}  // namespace Core

#endif
