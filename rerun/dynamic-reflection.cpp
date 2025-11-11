#include "capnp_schemas/std_msgs.capnp.h"   // genera los tipos estáticos
#include <capnp/dynamic.h>
#include <capnp/message.h>
#include <capnp/schema.h>
#include <capnp/serialize-packed.h>
#include <cstdint>
#include <cstring>
#include <iostream>

using namespace capnp;

void dynamicExample(int fd, StructSchema schema) {
    MallocMessageBuilder message;

    DynamicStruct::Builder root = message.initRoot<DynamicStruct>(schema);

    // Vamos a usar el tipo “Header” para este ejemplo dinámico,
    // pero lo hacemos por reflexión (“sin saber de antemano los campos”).

    root.set("seq", 42u);
    root.set("stampSec", 1639999999ull);
    root.set("stampNsec", 123456u);
    root.set("frameId", "map");

    // Serializamos el mensaje empaquetado
    writePackedMessageToFd(fd, message);
}

void dynamicReadExample(int fd, StructSchema schema) {
    PackedFdMessageReader reader(fd);

    DynamicStruct::Reader root = reader.getRoot<DynamicStruct>(schema);

    // Iteramos los campos de forma dinámica
    for (auto field : schema.getFields()) {
        auto name = field.getProto().getName().cStr();
        auto type = field.getType();
        std::cout << name << " = ";

        switch (type.which()) {
            case schema::Type::Which::VOID:
                std::cout << "(void)";
                break;
            case schema::Type::Which::UINT8:
            case schema::Type::Which::UINT16:
            case schema::Type::Which::UINT32:
            case schema::Type::Which::UINT64:
                std::cout << root.get(field).as<uint64_t>();
                break;
            case schema::Type::Which::TEXT:
                std::cout << "\"" << root.get(field).as<Text>().cStr() << "\"";
                break;
            // Aquí añadirías otros tipos según lo necesites
            default:
                std::cout << "(otro tipo)";
        }
        std::cout << "\n";
    }
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: <write|read>\n";
        return 1;
    }

    // Obtenemos el esquema del tipo Header
    StructSchema headerSchema = Schema::from<::Header>();

    if (strcmp(argv[1], "write") == 0) {
        dynamicExample(1, headerSchema);
    } else if (strcmp(argv[1], "read") == 0) {
        dynamicReadExample(0, headerSchema);
    } else {
        std::cerr << "Invalid arg: " << argv[1] << "\n";
        return 1;
    }

    return 0;
}
