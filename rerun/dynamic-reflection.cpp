#include "dynamic_reflection.hpp"

#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace {

void print_field(const DynamicReflection::ReflectedField& field,
                 int indent = 0) {
    const std::string pad(indent, ' ');
    if (field.is_leaf()) {
        std::cout << pad << field.name << ": " << field.value << '\n';
        return;
    }

    std::cout << pad << field.name << ":" << '\n';
    for (const auto& child : field.children) {
        print_field(child, indent + 2);
    }
}

void print_fields(const std::vector<DynamicReflection::ReflectedField>& fields,
                  int indent = 0) {
    for (const auto& field : fields) {
        print_field(field, indent);
    }
}

void print_metadata(const Core::EnvelopeMetadata& metadata) {
    if (!metadata.present) {
        std::cout << "Metadata: (not available)\n";
        return;
    }

    std::cout << "Metadata:" << '\n'
              << "  topic: " << metadata.topic << '\n'
              << "  typeId: " << metadata.typeId << '\n'
              << "  typeName: " << metadata.typeName << '\n'
              << "  timestampUsec: " << metadata.timestampUsec << '\n'
              << "  schemaPath: "
              << (metadata.schemaPath.empty() ? "<none>" : metadata.schemaPath)
              << '\n'
              << "  schemaTextBytes: " << metadata.schemaText.size() << '\n';
}

}  // namespace

int main(int argc, char* argv[]) {
    class AppArgumentParser : public Core::BaseArgumentParser {
     public:
        AppArgumentParser(int argc, char** argv)
            : Core::BaseArgumentParser(argc, argv) {}

        void parse_args() { this->parse(); }
    };

    AppArgumentParser args(argc, argv);
    args.add_argument("--schema").help("Path to the .capnp file");
    args.add_argument("--type").help("Name of the struct to inspect");
    args.add_argument("--topic")
        .default_value("telemetry")
        .help("Topic to read messages from for reflection");
    args.parse_args();

    auto schema = args.present("--schema");
    auto type = args.present("--type");
    auto topic = args.get_argument("--topic");

    DynamicReflection app(topic, std::move(schema), std::move(type));

    std::cout << "Waiting for the next message on topic '" << topic << "'..."
              << std::endl;

    const auto reflected = app.wait_for_message();

    print_metadata(reflected.metadata);
    std::cout << "Fields:" << '\n';
    print_fields(reflected.fields, 2);

    return 0;
}
