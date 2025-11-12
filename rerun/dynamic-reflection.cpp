#include <capnp/dynamic.h>
#include <capnp/schema-parser.h>
#include <capnp/schema.h>
#include <capnp/serialize-packed.h>
#include <kj/array.h>
#include <kj/io.h>
#include <kj/string.h>
#include "dynamic-reflection.hpp"
#include <functional>
#include <sstream>
#include <string>
#include <memory>

using namespace capnp;

static constexpr const char* kSchemaRoot = "messages/schemas";

static std::string printValue(DynamicValue::Reader value, int indent = 0) {
    std::ostringstream out;
    std::string pad(indent, ' ');
    switch (value.getType()) {
        case DynamicValue::VOID:
            out << pad << "(void)";
            break;
        case DynamicValue::BOOL:
            out << pad << (value.as<bool>() ? "true" : "false");
            break;
        case DynamicValue::INT:
            out << pad << value.as<int64_t>();
            break;
        case DynamicValue::UINT:
            out << pad << value.as<uint64_t>();
            break;
        case DynamicValue::FLOAT:
            out << pad << value.as<double>();
            break;
        case DynamicValue::TEXT:
            out << pad << "\"" << value.as<Text>().cStr() << "\"";
            break;
        case DynamicValue::STRUCT: {
            auto s = value.as<DynamicStruct>();
            out << pad << "{\n";
            for (auto field : s.getSchema().getFields()) {
                if (!s.has(field)) continue;
                out << pad << "  " << field.getProto().getName().cStr() << ": "
                    << printValue(s.get(field), indent + 4) << "\n";
            }
            out << pad << "}";
            break;
        }
        case DynamicValue::LIST: {
            auto l = value.as<DynamicList>();
            out << pad << "[\n";
            for (auto e : l) {
                out << printValue(e, indent + 4) << "\n";
            }
            out << pad << "]";
            break;
        }
        default:
            out << pad << "(unsupported)";
    }
    return out.str();
}

static StructSchema loadStructSchema(SchemaParser& parser, const char* schemaPath, const char* typeName) {
    kj::ArrayPtr<const kj::StringPtr> importPath;
    auto fileSchema = parser.parseDiskFile(kj::StringPtr(schemaPath), kj::StringPtr(schemaPath),
                                           importPath);

    ParsedSchema current = fileSchema;
    std::string name(typeName);
    size_t start = 0;
    while (start < name.size()) {
        auto dot = name.find('.', start);
        auto part = name.substr(start, dot == std::string::npos ? std::string::npos : dot - start);
        KJ_REQUIRE(!part.empty(), "Invalid type name", name);
        current = current.getNested(part.c_str());
        if (dot == std::string::npos) {
            break;
        }
        start = dot + 1;
    }

    return current.asStruct();
}

DynamicReflection::DynamicReflection(Core::ArgumentParser args)
    : Core::Vertex(args) {
    auto schema_arg = this->_args.get_argument("--schema");
    _schema_path = std::string(kSchemaRoot) + "/" + schema_arg;
    _type_name = this->_args.get_argument("--type");
    _topic = this->_args.get_argument("--topic");
    try {
        _schema = loadStructSchema(_parser, _schema_path.c_str(), _type_name.c_str());
    } catch (const kj::Exception& ex) {
        this->_logger.error("Failed to load schema %s (%s): %s",
                            _schema_path.c_str(), _type_name.c_str(),
                            ex.getDescription().cStr());
        throw;
    }
    _topic_sub = this->create_subscriber<capnp::AnyPointer>(
        _topic, std::bind(&DynamicReflection::message_cb, this, std::placeholders::_1));
}

void DynamicReflection::run() {
    this->_logger.info(
        "Listening to topic '%s' and reflecting messages as %s",
        _topic.c_str(), _type_name.c_str());
    Core::Vertex::run();
}

void DynamicReflection::message_cb(const Core::IncomingMessage<capnp::AnyPointer>& msg) {
    try {
        auto pointer = msg.content;
        auto root = pointer.getAs<DynamicStruct>(_schema);
        auto dump = printValue(root);
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

int main(int argc, char* argv[]) {
    Core::BaseArgumentParser args(argc, argv);
    args.add_argument("--schema").required().help("Path to the .capnp file");
    args.add_argument("--type").required().help("Name of the struct to inspect");
    args.add_argument("--topic")
        .default_value("telemetry")
        .help("Topic to read messages from for reflection");
    DynamicReflection app(args);
    app.run();
    return 0;
}
