#include <iostream>
#include <fcntl.h>
#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include <capnp_schemas/registry.capnp.h>

int main() {
    auto fd = open("registry_request.bin", O_WRONLY | O_CREAT | O_TRUNC);
    if (!fd) {
        std::cerr << "Failed to open file for writing." << std::endl;
        return 1;
    }

    ::capnp::MallocMessageBuilder message;
    auto request = message.initRoot<RegistryRequest>();
    request.setType(RequestType::ADD_NODE);
    auto addNode = request.initAddNode();
    addNode.setPath("example/path");
    addNode.setPort(4020);

    ::capnp::writePackedMessageToFd(fd, message);
    return 0;
}