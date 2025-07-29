#include "nodes_yaml_parser.hpp"
#include <yaml-cpp/yaml.h>
#include <sstream>
#include <iostream>



NodesYamlParser::NodesYamlParser(const std::string& filename) {
    YAML::Node config = YAML::LoadFile(filename);
    std::ostringstream oss;
    for (const auto& exe : config["executables"]) {
        ExecutableArgs execArgs;
        execArgs.name = exe["name"].as<std::string>();
        std::vector<std::string> arg_line;
        if (exe["args"]) {
            const auto& args = exe["args"];
            if (args["flags"]) {
                for (const auto& flag : args["flags"]) {
                    execArgs.flags.push_back(flag.as<std::string>());
                    arg_line.push_back(flag.as<std::string>());
                }
            }
            if (args["options"]) {
                for (const auto& opt : args["options"]) {
                    execArgs.options[opt.first.as<std::string>()] = opt.second.as<std::string>();
                    arg_line.push_back(opt.first.as<std::string>());
                    arg_line.push_back(opt.second.as<std::string>());
                }
            }
            if (args["positionals"]) {
                for (const auto& positional : args["positionals"]) {
                    execArgs.positionals.push_back(positional.as<std::string>());
                    arg_line.push_back(positional.as<std::string>());
                }
            }
        }
        executables.push_back(execArgs);
        args_lines[execArgs.name] = arg_line;
        // Construir string YAML en el mismo ciclo
        oss << "Executable: " << execArgs.name << '\n';
        if (!execArgs.flags.empty()) {
            oss << "  Flags:";
            for (const auto& flag : execArgs.flags) {
                oss << " " << flag;
            }
            oss << '\n';
        }
        if (!execArgs.options.empty()) {
            oss << "  Options:" << '\n';
            for (const auto& opt : execArgs.options) {
                oss << "    " << opt.first << ": " << opt.second << '\n';
            }
        }
        if (!execArgs.positionals.empty()) {
            oss << "  Positionals:";
            for (const auto& pos : execArgs.positionals) {
                oss << " " << pos;
            }
            oss << '\n';
        }
    }
    yaml_str = oss.str();
}

const std::string& NodesYamlParser::get_yaml() const {
    return yaml_str;
}

std::vector<std::string> NodesYamlParser::get_executables() const {
    std::vector<std::string> names;
    for (const auto& exe : executables) {
        names.push_back(exe.name);
    }
    return names;
}

const std::vector<std::string>& NodesYamlParser::get_args_line(const std::string& exec_name) const {
    static const std::vector<std::string> empty;
    auto it = args_lines.find(exec_name);
    if (it != args_lines.end()) return it->second;
    return empty;
}
