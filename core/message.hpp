#ifndef MESSAGE_HPP
#define MESSAGE_HPP

#include <capnp/any.h>
#include <capnp/message.h>
#include <capnp/schema.h>
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

struct EnvelopeMetadata {
  bool present = false;
  uint64_t typeId = 0;
  uint64_t timestampUsec = 0;
  std::string typeName;
  std::string topic;
  std::string schemaPath;
  std::string schemaText;
};
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

    capnp::Schema schema = capnp::Schema::from<T>();
    auto proto = schema.getProto();
    auto displayName = proto.getDisplayName();
    std::string qualified_name;
    if (displayName.size() > 0) {
      qualified_name.assign(displayName.cStr(), displayName.size());
    } else {
      qualified_name = typeid(T).name();
    }

    std::string type_name = qualified_name;
    auto name_sep = type_name.find_last_of(':');
    if (name_sep == std::string::npos) {
      name_sep = type_name.find_last_of('.');
    }
    if (name_sep != std::string::npos && name_sep + 1 < type_name.size()) {
      type_name = type_name.substr(name_sep + 1);
    }
    envelope.setTypeName(type_name);

    std::string schema_path;
    auto colon_pos = qualified_name.find(':');
    if (colon_pos != std::string::npos) {
      schema_path = qualified_name.substr(0, colon_pos);
    } else {
      auto dot_pos = qualified_name.find('.');
      if (dot_pos != std::string::npos) {
        schema_path = qualified_name.substr(0, dot_pos) + ".capnp";
      }
    }
    envelope.setSchemaPath(schema_path);
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
        metadata.present = true;
        metadata.typeId = envelope.getTypeId();
        metadata.timestampUsec = envelope.getTimestampUsec();
        auto typeName = envelope.getTypeName();
        metadata.typeName = std::string(typeName.cStr(), typeName.size());
        auto topic = envelope.getTopic();
        metadata.topic = std::string(topic.cStr(), topic.size());
        auto schemaPath = envelope.getSchemaPath();
        metadata.schemaPath = std::string(schemaPath.cStr(), schemaPath.size());
        auto schemaText = envelope.getSchemaText();
        metadata.schemaText = std::string(schemaText.cStr(), schemaText.size());
        envelopeAvailable = true;
        return;
      }
    } catch (const ::kj::Exception&) {
      // Fall back to raw struct parsing below.
    }
    metadata = EnvelopeMetadata{};
    stream = std::make_unique<::kj::ArrayInputStream>(ptr);
    reader = std::make_unique<::capnp::PackedMessageReader>(*stream);
    content = reader->getRoot<T>();
  }

 public:
  typename T::Reader content;
  EnvelopeMetadata metadata;

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
        envelopeAvailable(other.envelopeAvailable),
        metadata(other.metadata) {
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
        envelopeAvailable(false),
        metadata() {
    parse();
  }
};

}  // namespace Core

#endif
