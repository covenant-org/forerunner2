#include "nodes_yaml_parser.hpp"
#include <iostream>
#include <sstream>
#include <yaml-cpp/yaml.h>

NodesYamlParser::NodesYamlParser(const std::string& filename) {
  YAML::Node config = YAML::LoadFile(filename);
  std::ostringstream oss;
  for (const auto& exe : config["executables"]) {
    ExecutableArgs exec_args;
    exec_args.name = exe["name"].as<std::string>();
    std::vector<std::string> arg_line;
    if (exe["args"]) {
      const auto& args = exe["args"];
      if (args["flags"]) {
        for (const auto& flag : args["flags"]) {
          exec_args.flags.push_back(flag.as<std::string>());
          arg_line.push_back(flag.as<std::string>());
        }
      }
      if (args["options"]) {
        for (const auto& opt : args["options"]) {
          exec_args.options[opt.first.as<std::string>()] =
              opt.second.as<std::string>();
          arg_line.push_back(opt.first.as<std::string>());
          arg_line.push_back(opt.second.as<std::string>());
        }
      }
      if (args["positionals"]) {
        for (const auto& positional : args["positionals"]) {
          exec_args.positionals.push_back(positional.as<std::string>());
          arg_line.push_back(positional.as<std::string>());
        }
      }
    }
    executables.push_back(exec_args);
    args_lines[exec_args.name] = arg_line;
    // Construir string YAML en el mismo ciclo
    oss << "Executable: " << exec_args.name << '\n';
    if (!exec_args.flags.empty()) {
      oss << "  Flags:";
      for (const auto& flag : exec_args.flags) {
        oss << " " << flag;
      }
      oss << '\n';
    }
    if (!exec_args.options.empty()) {
      oss << "  Options:" << '\n';
      for (const auto& opt : exec_args.options) {
        oss << "    " << opt.first << ": " << opt.second << '\n';
      }
    }
    if (!exec_args.positionals.empty()) {
      oss << "  Positionals:";
      for (const auto& pos : exec_args.positionals) {
        oss << " " << pos;
      }
      oss << '\n';
    }
  }
  yaml_str = oss.str();
}

const std::string& NodesYamlParser::get_yaml() const { return yaml_str; }

std::vector<std::string> NodesYamlParser::get_executables() const {
  std::vector<std::string> names;
  for (const auto& exe : executables) {
    names.push_back(exe.name);
  }
  return names;
}

const std::vector<std::string>& NodesYamlParser::get_args_line(
    const std::string& exec_name) const {
  static const std::vector<std::string> empty;
  auto it = args_lines.find(exec_name);
  if (it != args_lines.end()) return it->second;
  return empty;
}
