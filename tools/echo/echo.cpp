#include "dynamic_reflection.hpp"

#include <iostream>
#include <optional>
#include <string>
#include <utility>
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

int main(int argc, char** argv) {
    class AppArgumentParser : public Core::BaseArgumentParser {
     public:
        AppArgumentParser(int argc, char** argv)
            : Core::BaseArgumentParser(argc, argv) {}

        void parse_args() { this->parse(); }
    };

    AppArgumentParser args(argc, argv);
    args.add_argument("--topic")
        .help("Topic to echo messages from")
        .required();
    args.add_argument("-n", "--count")
        .help("Number of messages to echo before exiting (0 = infinite)")
        .scan<'i', int>()
        .default_value(0);
    args.add_argument("-m", "--metadata")
        .help("Print metadata before fields")
        .default_value(false)
        .implicit_value(true);
    args.add_argument("--schema")
        .help("Override path to the .capnp schema used for reflection");
    args.add_argument("--type")
        .help("Override name of the struct to inspect");
    args.parse_args();

    auto topic = args.get_argument<std::string>("topic");
    auto schema = args.present("--schema");
    auto type = args.present("--type");
    const auto count = args.get_argument<int>("--count");
    const auto show_metadata = args.get_argument<bool>("--metadata");

    DynamicReflection reflector(topic, std::move(schema), std::move(type));

    std::cout << "Echo listening on '" << topic << "'" << std::endl;

    int echoed = 0;
    while (count == 0 || echoed < count) {
        const auto reflected = reflector.wait_for_message();
        if (show_metadata) {
            print_metadata(reflected.metadata);
        }
        std::cout << reflected.metadata.typeName << ':' << '\n';
        print_fields(reflected.fields, 2);
        std::cout << std::flush;
        ++echoed;
    }

    return 0;
}
