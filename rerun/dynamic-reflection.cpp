#include "dynamic_reflection.hpp"

int main(int argc, char* argv[]) {
    class AppArgumentParser : public Core::BaseArgumentParser {
     public:
        AppArgumentParser(int argc, char** argv)
            : Core::BaseArgumentParser(argc, argv) {}

        void parse_args() { this->parse(); }
    };

    AppArgumentParser args(argc, argv);
    args.add_argument("--schema").required().help("Path to the .capnp file");
    args.add_argument("--type").required().help("Name of the struct to inspect");
    args.add_argument("--topic")
        .default_value("telemetry")
        .help("Topic to read messages from for reflection");
    args.parse_args();

    auto schema = args.get_argument("--schema");
    auto type = args.get_argument("--type");
    auto topic = args.get_argument("--topic");

    DynamicReflection app(schema, type, topic);
    app.run();
    return 0;
}
