#include <yaml-cpp/yaml.h>
#include <iostream>
#include <vector>
#include <map>
#include <string>

class NodesYamlParser {
public:
    struct ExecutableArgs {
        std::string name;
        std::vector<std::string> flags;
        std::map<std::string, std::string> options;
        std::vector<std::string> positionals;
    };

    std::vector<ExecutableArgs> executables;

    NodesYamlParser(const std::string& filename) {
        YAML::Node config = YAML::LoadFile(filename);
        for (const auto& exe : config["executables"]) {
            ExecutableArgs eargs;
            eargs.name = exe["name"].as<std::string>();
            const auto& args = exe["args"];
            if (args["flags"]) {
                for (const auto& flag : args["flags"]) {
                    eargs.flags.push_back(flag.as<std::string>());
                }
            }
            if (args["options"]) {
                for (const auto& opt : args["options"]) {
                    eargs.options[opt.first.as<std::string>()] = opt.second.as<std::string>();
                }
            }
            if (args["positionals"]) {
                for (const auto& pos : args["positionals"]) {
                    eargs.positionals.push_back(pos.as<std::string>());
                }
            }
            executables.push_back(eargs);
        }
    }


    std::string yaml() const {
        std::ostringstream oss;
        for (const auto& exe : executables) {
            oss << "Executable: " << exe.name << '\n';
            // Flags
            if (!exe.flags.empty()) {
                oss << "  Flags:";
                for (const auto& flag : exe.flags) {
                    oss << " " << flag;
                }
                oss << '\n';
            }
            // Options
            if (!exe.options.empty()) {
                oss << "  Options:" << '\n';
                for (const auto& opt : exe.options) {
                    oss << "    " << opt.first << ": " << opt.second << '\n';
                }
            }
            // Positionals
            if (!exe.positionals.empty()) {
                oss << "  Positionals:";
                for (const auto& pos : exe.positionals) {
                    oss << " " << pos;
                }
                oss << '\n';
            }
        }
        return oss.str();
    }
};

int main() {
    NodesYamlParser parser("args_template.yaml");
    std::cout << "--- Imprimiendo yaml ---" << std::endl;
    std::cout << parser.yaml();
    return 0;
}