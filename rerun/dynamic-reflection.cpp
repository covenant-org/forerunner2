#include <capnp/dynamic.h>
#include <capnp/message.h>
#include <capnp/schema-parser.h>
#include <capnp/schema.h>
#include <capnp/serialize-packed.h>
#include <kj/array.h>
#include <kj/debug.h>
#include <kj/string.h>
#include <iostream>
#include <string>
#include <unistd.h>

using namespace capnp;

void printValue(DynamicValue::Reader value, int indent = 0) {
    std::string pad(indent, ' ');
    switch (value.getType()) {
        case DynamicValue::VOID:
            std::cout << pad << "(void)";
            break;
        case DynamicValue::BOOL:
            std::cout << pad << (value.as<bool>() ? "true" : "false");
            break;
        case DynamicValue::INT:
            std::cout << pad << value.as<int64_t>();
            break;
        case DynamicValue::UINT:
            std::cout << pad << value.as<uint64_t>();
            break;
        case DynamicValue::FLOAT:
            std::cout << pad << value.as<double>();
            break;
        case DynamicValue::TEXT:
            std::cout << pad << "\"" << value.as<Text>().cStr() << "\"";
            break;
        case DynamicValue::STRUCT: {
            auto s = value.as<DynamicStruct>();
            std::cout << pad << "{\n";
            for (auto field : s.getSchema().getFields()) {
                if (!s.has(field)) continue;
                std::cout << pad << "  " << field.getProto().getName().cStr() << ": ";
                printValue(s.get(field), indent + 4);
                std::cout << "\n";
            }
            std::cout << pad << "}";
            break;
        }
        case DynamicValue::LIST: {
            auto l = value.as<DynamicList>();
            std::cout << pad << "[\n";
            for (auto e : l) {
                printValue(e, indent + 4);
                std::cout << "\n";
            }
            std::cout << pad << "]";
            break;
        }
        default:
            std::cout << pad << "(unsupported)";
    }
}

static StructSchema loadStructSchema(const char* schemaPath, const char* typeName) {
    SchemaParser parser;

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

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: rerun_dynamic_reflection <schema.capnp> <TypeName>\n";
        return 1;
    }

    try {
        auto schema = loadStructSchema(argv[1], argv[2]);
        PackedFdMessageReader reader(STDIN_FILENO);
        auto root = reader.getRoot<DynamicStruct>(schema);
        printValue(root);
        std::cout << std::endl;
    } catch (const kj::Exception& ex) {
        std::cerr << "Error: " << ex.getDescription().cStr() << "\n";
        return 1;
    }

    return 0;
}
