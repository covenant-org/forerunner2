#include "capnp/dynamic.h"
#include "capnp/schema-loader.h"
#include "capnp/serialize-packed.h"
#include <capnp/message.h>
#include <iostream>
#include <fstream>

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

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: <schemaId or schemaFile>\n";
        return 1;
    }

    // 1. Cargar esquema dinámicamente
    SchemaLoader loader;
    // Si tienes un archivo .capnp o archivo .capnp.bin de esquema:
    // loader.loadSchemaFile("std_msgs.capnp");
    // Para simplicidad, supongamos usamos un esquema pre-generado (esto debes adaptar)

    // 2. Leer mensaje desde stdin (fd 0) por ejemplo
    MallocMessageBuilder message;
    // En la práctica usarías PackedFdMessageReader reader(0);
    // Reader root = reader.getRoot<DynamicStruct>(schema);

    // 3. Impresion genérica
    // printValue(root);

    return 0;
}
