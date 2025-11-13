#ifndef CORE_DYNAMIC_REFLECTION_HPP
#define CORE_DYNAMIC_REFLECTION_HPP

#include "vertex.hpp"
#include <capnp/any.h>
#include <capnp/dynamic.h>
#include <capnp/schema-parser.h>
#include <condition_variable>
#include <functional>
#include <kj/array.h>
#include <kj/common.h>
#include <kj/filesystem.h>
#include <kj/string.h>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <stdexcept>
#include <utility>
#include <vector>

class DynamicReflection : public Core::Vertex {
 public:
    struct ReflectedField {
        std::string name;
        bool hasValue = false;
        std::string value;
        std::vector<ReflectedField> children;

        bool is_leaf() const { return hasValue; }
    };

    struct ReflectedMessage {
        Core::EnvelopeMetadata metadata;
        std::vector<ReflectedField> fields;
    };

    DynamicReflection(const std::string& topic,
                      std::optional<std::string> schema = std::nullopt,
                      std::optional<std::string> type = std::nullopt);
    // TODO: Con el puro topico ya podria extraer los otros 2 (Schema y type)

    void run() override;

    ReflectedMessage wait_for_message();

    static std::vector<ReflectedField> reflect(
        const capnp::DynamicStruct::Reader& root);

 private:
    static Core::ArgumentParser make_default_args() {
        static char program_name[] = "dynamic-reflection";
        static char* argv[] = {program_name};
        Core::BaseArgumentParser parser(1, argv);
        return parser;
    }

    void message_cb(const Core::IncomingMessage<capnp::AnyPointer>& msg);

    static ReflectedField build_field(std::string name,
                                      capnp::DynamicValue::Reader value);

    static std::string scalar_to_string(capnp::DynamicValue::Reader value);

    bool load_schema_from_path(const std::string& schema_path,
                               const std::string& type_name,
                               bool log_failure);

    std::string _topic;
    std::optional<std::string> _configured_schema;
    std::optional<std::string> _configured_type;
    capnp::StructSchema _schema;
    std::shared_ptr<Core::Subscriber<capnp::AnyPointer>> _topic_sub;
    capnp::SchemaParser _parser;
    kj::Own<kj::Filesystem> _filesystem;
    const kj::Directory* _base_directory = nullptr;
    bool _schema_ready = false;

    std::mutex _mutex;
    std::condition_variable _cv;
    std::optional<ReflectedMessage> _latest;
};

inline DynamicReflection::DynamicReflection(const std::string& topic,
                                            std::optional<std::string> schema,
                                            std::optional<std::string> type)
    : Core::Vertex(make_default_args()),
      _topic(topic),
      _configured_schema(std::move(schema)),
      _configured_type(std::move(type)),
      _filesystem(kj::newDiskFilesystem()) {
    _base_directory = &_filesystem->getCurrent();
    if (_configured_schema && _configured_type) {
        if (!load_schema_from_path(*_configured_schema, *_configured_type, true)) {
            throw std::runtime_error("Failed to preload schema");
        }
    }

    _topic_sub = this->create_subscriber<capnp::AnyPointer>(
        _topic, std::bind(&DynamicReflection::message_cb, this, std::placeholders::_1));
}

inline bool DynamicReflection::load_schema_from_path(
    const std::string& schema_path,
    const std::string& type_name,
    bool log_failure) {
    std::vector<std::string> candidates;
    candidates.push_back(schema_path);
    if (schema_path.rfind("messages/schemas/", 0) != 0) {
        candidates.push_back("messages/schemas/" + schema_path);
    }

    for (size_t i = 0; i < candidates.size(); ++i) {
        const bool last_attempt = (i + 1 == candidates.size());
        try {
            kj::ArrayPtr<const kj::ReadableDirectory* const> import_dirs;
            auto file_schema = _parser.parseFromDirectory(
                *_base_directory, kj::Path::parse(candidates[i].c_str()), import_dirs);

            capnp::ParsedSchema current = file_schema;
            std::string qualified(type_name);
            size_t start = 0;
            while (start < qualified.size()) {
                const auto dot = qualified.find('.', start);
                const auto part = qualified.substr(
                    start,
                    dot == std::string::npos ? std::string::npos : dot - start);
                KJ_REQUIRE(!part.empty(), "Invalid type name", qualified);
                current = current.getNested(part.c_str());
                if (dot == std::string::npos) break;
                start = dot + 1;
            }

            _schema = current.asStruct();
            _schema_ready = true;
            _configured_schema = candidates[i];
            _configured_type = type_name;
            this->_logger.info("Loaded schema %s (%s)",
                                candidates[i].c_str(),
                                type_name.c_str());
            return true;
        } catch (const kj::Exception& ex) {
            const auto* message = ex.getDescription().cStr();
            const bool should_log = last_attempt ? log_failure : false;
            if (should_log) {
                this->_logger.error("Failed to load schema %s (%s): %s",
                                    candidates[i].c_str(),
                                    type_name.c_str(),
                                    message);
            } else {
                this->_logger.debug("Failed to load schema %s (%s): %s",
                                     candidates[i].c_str(),
                                     type_name.c_str(),
                                     message);
            }
        }
    }

    return false;
}

inline void DynamicReflection::run() {
    this->_logger.info("Listening to topic '%s'", _topic.c_str());
    Core::Vertex::run();
}

inline void DynamicReflection::message_cb(
    const Core::IncomingMessage<capnp::AnyPointer>& msg) {
    try {
        if (!_schema_ready) {
            if (!load_schema_from_path(msg.metadata.schemaPath,
                                       msg.metadata.typeName,
                                       true)) {
                return;
            }
        }

        auto pointer = msg.content;
        auto root = pointer.getAs<capnp::DynamicStruct>(_schema);
        ReflectedMessage captured{msg.metadata, reflect(root)};
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _latest = std::move(captured);
        }
        _cv.notify_all();
    } catch (const kj::Exception& ex) {
        this->_logger.error("Error reflecting message: %s",
                            ex.getDescription().cStr());
    }
}

inline std::vector<DynamicReflection::ReflectedField> DynamicReflection::reflect(
    const capnp::DynamicStruct::Reader& root) {
    std::vector<ReflectedField> fields;
    for (auto field : root.getSchema().getFields()) {
        if (!root.has(field)) continue;
        const auto proto = field.getProto().getName();
        fields.push_back(
            build_field(std::string(proto.cStr(), proto.size()),
                        root.get(field)));
    }
    return fields;
}

inline DynamicReflection::ReflectedField DynamicReflection::build_field(
    std::string name,
    capnp::DynamicValue::Reader value) {
    ReflectedField node;
    node.name = std::move(name);
    switch (value.getType()) {
        case capnp::DynamicValue::STRUCT: {
            node.children = reflect(value.as<capnp::DynamicStruct>());
            break;
        }
        case capnp::DynamicValue::LIST: {
            auto list = value.as<capnp::DynamicList>();
            node.children.reserve(list.size());
            for (size_t index = 0; index < list.size(); ++index) {
                node.children.push_back(
                    build_field("[" + std::to_string(index) + "]",
                                list[index]));
            }
            break;
        }
        default: {
            node.hasValue = true;
            node.value = scalar_to_string(value);
            break;
        }
    }
    return node;
}

inline std::string DynamicReflection::scalar_to_string(
    capnp::DynamicValue::Reader value) {
    switch (value.getType()) {
        case capnp::DynamicValue::VOID:
            return "(void)";
        case capnp::DynamicValue::BOOL:
            return value.as<bool>() ? "true" : "false";
        case capnp::DynamicValue::INT:
            return std::to_string(value.as<int64_t>());
        case capnp::DynamicValue::UINT:
            return std::to_string(value.as<uint64_t>());
        case capnp::DynamicValue::FLOAT: {
            std::ostringstream ss;
            ss << value.as<double>();
            return ss.str();
        }
        case capnp::DynamicValue::TEXT: {
            auto text = value.as<capnp::Text>();
            return std::string(text.cStr(), text.size());
        }
        case capnp::DynamicValue::ENUM: {
            auto dyn_enum = value.as<capnp::DynamicEnum>();
            KJ_IF_MAYBE(enumerant, dyn_enum.getEnumerant()) {
                auto name = enumerant->getProto().getName();
                return std::string(name.cStr(), name.size());
            }
            return std::to_string(dyn_enum.getRaw());
        }
        case capnp::DynamicValue::DATA: {
            auto data = value.as<capnp::Data>();
            return "data[" + std::to_string(data.size()) + "]";
        }
        default:
            return "(unsupported)";
    }
}

inline DynamicReflection::ReflectedMessage DynamicReflection::wait_for_message() {
    std::unique_lock<std::mutex> lock(_mutex);
    _cv.wait(lock, [this] { return _latest.has_value(); });
    ReflectedMessage result = std::move(*_latest);
    _latest.reset();
    return result;
}

#endif  // CORE_DYNAMIC_REFLECTION_HPP
