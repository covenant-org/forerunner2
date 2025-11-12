#ifndef CORE_DYNAMIC_REFLECTION_HPP
#define CORE_DYNAMIC_REFLECTION_HPP

#include "vertex.hpp"
#include <capnp/any.h>
#include <capnp/dynamic.h>
#include <capnp/schema-parser.h>
#include <functional>
#include <kj/array.h>
#include <kj/string.h>
#include <memory>
#include <sstream>
#include <string>

class DynamicReflection : public Core::Vertex {
 public:
    DynamicReflection(const std::string& schema,
                      const std::string& type,
                      const std::string& topic);
    // TODO: Con el puro topico ya podria extraer los otros 2 (Schema y type) 

    void run() override;

 private:
    static Core::ArgumentParser make_default_args() {
        static char program_name[] = "dynamic-reflection";
        static char* argv[] = {program_name};
        return Core::ArgumentParser(1, argv);
    }

    static std::string format_value(capnp::DynamicValue::Reader value,
                                    int indent = 0) {
        std::ostringstream out;
        std::string pad(indent, ' ');
        switch (value.getType()) {
            case capnp::DynamicValue::VOID:
                out << pad << "(void)";
                break;
            case capnp::DynamicValue::BOOL:
                out << pad << (value.as<bool>() ? "true" : "false");
                break;
            case capnp::DynamicValue::INT:
                out << pad << value.as<int64_t>();
                break;
            case capnp::DynamicValue::UINT:
                out << pad << value.as<uint64_t>();
                break;
            case capnp::DynamicValue::FLOAT:
                out << pad << value.as<double>();
                break;
            case capnp::DynamicValue::TEXT:
                out << pad << '"' << value.as<capnp::Text>().cStr() << '"';
                break;
            case capnp::DynamicValue::STRUCT: {
                auto dynamic_struct = value.as<capnp::DynamicStruct>();
                out << pad << "{\n";
                for (auto field : dynamic_struct.getSchema().getFields()) {
                    if (!dynamic_struct.has(field)) continue;
                    out << pad << "  "
                        << field.getProto().getName().cStr() << ": "
                        << format_value(dynamic_struct.get(field), indent + 4)
                        << "\n";
                }
                out << pad << "}";
                break;
            }
            case capnp::DynamicValue::LIST: {
                auto dynamic_list = value.as<capnp::DynamicList>();
                out << pad << "[\n";
                for (auto element : dynamic_list) {
                    out << format_value(element, indent + 4) << "\n";
                }
                out << pad << "]";
                break;
            }
            default:
                out << pad << "(unsupported)";
        }
        return out.str();
    }

    static capnp::StructSchema load_struct_schema(capnp::SchemaParser& parser,
                                                  const char* schema_path,
                                                  const char* type_name) {
        kj::ArrayPtr<const kj::StringPtr> import_path;
        auto file_schema = parser.parseDiskFile(kj::StringPtr(schema_path),
                                                kj::StringPtr(schema_path),
                                                import_path);

        capnp::ParsedSchema current = file_schema;
        std::string qualified(type_name);
        size_t start = 0;
        while (start < qualified.size()) {
            auto dot = qualified.find('.', start);
            auto part = qualified.substr(
                start, dot == std::string::npos ? std::string::npos : dot - start);
            KJ_REQUIRE(!part.empty(), "Invalid type name", qualified);
            current = current.getNested(part.c_str());
            if (dot == std::string::npos) {
                break;
            }
            start = dot + 1;
        }

        return current.asStruct();
    }

    void message_cb(const Core::IncomingMessage<capnp::AnyPointer>& msg);

    std::string _schema_path;
    std::string _type_name;
    std::string _topic;
    capnp::StructSchema _schema;
    std::shared_ptr<Core::Subscriber<capnp::AnyPointer>> _topic_sub;
    capnp::SchemaParser _parser;
};

inline DynamicReflection::DynamicReflection(const std::string& schema,
                                            const std::string& type,
                                            const std::string& topic)
    : Core::Vertex(make_default_args()),
      _schema_path(std::string("messages/schemas/") + schema),
      _type_name(type),
      _topic(topic) {
    try {
        _schema = load_struct_schema(_parser, _schema_path.c_str(), _type_name.c_str());
    } catch (const kj::Exception& ex) {
        this->_logger.error("Failed to load schema %s (%s): %s",
                            _schema_path.c_str(),
                            _type_name.c_str(),
                            ex.getDescription().cStr());
        throw;
    }

    _topic_sub = this->create_subscriber<capnp::AnyPointer>(
        _topic, std::bind(&DynamicReflection::message_cb, this, std::placeholders::_1));
}

inline void DynamicReflection::run() {
    this->_logger.info(
        "Listening to topic '%s' and reflecting messages as %s",
        _topic.c_str(),
        _type_name.c_str());
    Core::Vertex::run();
}

inline void DynamicReflection::message_cb(
    const Core::IncomingMessage<capnp::AnyPointer>& msg) {
    try {
        auto pointer = msg.content;
        auto root = pointer.getAs<capnp::DynamicStruct>(_schema);
        auto dump = format_value(root);
        if (msg.metadata.present) {
            this->_logger.info(
                "Message received (topic=%s typeId=%lu ts=%lu):\n%s",
                msg.metadata.topic.c_str(),
                static_cast<unsigned long>(msg.metadata.typeId),
                static_cast<unsigned long>(msg.metadata.timestampUsec),
                dump.c_str());
        } else {
            this->_logger.info("Message received:\n%s", dump.c_str());
        }
    } catch (const kj::Exception& ex) {
        this->_logger.error("Error reflecting message: %s",
                            ex.getDescription().cStr());
    }
}

#endif  // CORE_DYNAMIC_REFLECTION_HPP
