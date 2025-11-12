#ifndef MESSAGE_HPP
#define MESSAGE_HPP

#include <capnp/any.h>
#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include <capnp/serialize.h>
#include <capnp_schemas/envelope.capnp.h>
#include <kj/common.h>
#include <kj/exception.h>
#include <kj/io.h>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <typeinfo>
#include <zmq.hpp>

namespace Core {
class ISender {
 public:
  virtual ~ISender() = default;
  virtual uint32_t publish(::capnp::MallocMessageBuilder& builder) = 0;
  virtual uint32_t reset_connection(const std::string&) = 0;
};

template <typename T>
class OutgoingMessage {
 private:
  ::capnp::MallocMessageBuilder builder;
  ISender* sender;
  Envelope::Builder envelope;
  ::capnp::AnyPointer::Builder payload;

 public:
  typename T::Builder content;
  OutgoingMessage(ISender* sender, const std::string& topic)
      : builder(),
        sender(sender),
        envelope(builder.initRoot<Envelope>()),
        payload(envelope.initPayload()),
        content(payload.initAs<T>()) {
    const uint64_t micros =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count();
    envelope.setTimestampUsec(micros);
    envelope.setTypeId(T::_capnpPrivate::typeId);
    envelope.setSchemaPath("");
    envelope.setTypeName(typeid(T).name());
    envelope.setTopic(topic);
  }
  uint32_t publish() { return sender->publish(builder); }
  Envelope::Builder getEnvelope() { return envelope; }
};

template <typename T>
class IncomingMessage {
 public:
  std::string buffer;

 private:
  ::kj::ArrayPtr<unsigned char> ptr;
  std::unique_ptr<::kj::ArrayInputStream> stream;
  std::unique_ptr<::capnp::PackedMessageReader> reader;
  bool envelopeAvailable;
  Envelope::Reader envelope;
  ::capnp::AnyPointer::Reader payload;

  void parse() {
    envelopeAvailable = false;
    try {
      envelope = reader->getRoot<Envelope>();
      payload = envelope.getPayload();
      const bool validEnvelope = envelope.getTypeId() != 0 &&
                                 envelope.getTopic().size() > 0 &&
                                 !payload.isNull();
      if (validEnvelope) {
        content = payload.getAs<T>();
        envelopeAvailable = true;
        return;
      }
    } catch (const ::kj::Exception&) {
      // Fall back to raw struct parsing below.
    }
    stream = std::make_unique<::kj::ArrayInputStream>(ptr);
    reader = std::make_unique<::capnp::PackedMessageReader>(*stream);
    content = reader->getRoot<T>();
  }

 public:
  typename T::Reader content;

  bool hasEnvelope() const { return envelopeAvailable; }

  Envelope::Reader getEnvelope() const {
    if (!envelopeAvailable) {
      throw std::runtime_error("Envelope metadata not available");
    }
    return envelope;
  }

  ::capnp::AnyPointer::Reader getRawPayload() const {
    if (!envelopeAvailable) {
      throw std::runtime_error("Envelope metadata not available");
    }
    return payload;
  }

  IncomingMessage(const IncomingMessage& other)
      : buffer(other.buffer),
        ptr((unsigned char*)buffer.data(), buffer.size()),
        stream(std::make_unique<::kj::ArrayInputStream>(ptr)),
        reader(std::make_unique<::capnp::PackedMessageReader>(*stream)),
        envelopeAvailable(other.envelopeAvailable) {
    if (envelopeAvailable) {
      envelope = reader->getRoot<Envelope>();
      payload = envelope.getPayload();
      content = payload.getAs<T>();
    } else {
      content = reader->getRoot<T>();
    }
  }

  IncomingMessage(unsigned char* data, uint32_t size)
      : buffer((char*)data, size),
        ptr((unsigned char*)buffer.data(), size),
        stream(std::make_unique<::kj::ArrayInputStream>(ptr)),
        reader(std::make_unique<::capnp::PackedMessageReader>(*stream)),
        envelopeAvailable(false) {
    parse();
  }
};

}  // namespace Core

#endif
