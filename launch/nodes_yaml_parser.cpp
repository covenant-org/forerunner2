#include "nodes_yaml_parser.hpp"
#include <sstream>
#include <yaml-cpp/yaml.h>

NodesYamlParser::NodesYamlParser(const std::string& filename) {

  Core::Logger _logger(Core::LogLevel::INFO, "application.log",
                          "nodes_yaml_parser");
  _logger.set_classname("nodes_yaml_parser");
  
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
        const YAML::Node opts = args["options"];
        if (opts.IsMap()) {
          for (const auto& opt : opts) {
            std::string key = opt.first.as<std::string>();
            if (opt.second && !opt.second.IsNull()) {
              std::string val = opt.second.as<std::string>();
              exec_args.options[key] = val;
              arg_line.push_back(key);
              arg_line.push_back(val);
            } else {
              // Fallback: map key present but value is null. Try to split
              // the key into "key value" (handles malformed YAML like
              // "--port 5020" without a colon) or treat as flag.
              std::istringstream iss(key);
              std::string k, v;
              if (iss >> k) {
                if (iss >> v) {
                  exec_args.options[k] = v;
                  arg_line.push_back(k);
                  arg_line.push_back(v);
                } else {
                  exec_args.flags.push_back(k);
                  arg_line.push_back(k);
                }
              }
            }
          }
        } else if (opts.IsSequence()) {
          // Support sequence of scalars like: options: ["--port 5020", ...]
          for (const auto& n : opts) {
            std::string s = n.as<std::string>();
            std::istringstream iss(s);
            std::string key, val;
            if (iss >> key) {
              if (iss >> val) {
                exec_args.options[key] = val;
                arg_line.push_back(key);
                arg_line.push_back(val);
              } else {
                // single token, treat as flag without value
                exec_args.flags.push_back(key);
                arg_line.push_back(key);
              }
            }
          }
        } else if (opts.IsScalar()) {
          // Single scalar, try to split into key/value
          std::string s = opts.as<std::string>();
          std::istringstream iss(s);
          std::string key, val;
          if (iss >> key) {
            if (iss >> val) {
              exec_args.options[key] = val;
              arg_line.push_back(key);
              arg_line.push_back(val);
            } else {
              exec_args.flags.push_back(key);
              arg_line.push_back(key);
            }
          }
        }
      }
      if (args["positionals"]) {
        for (const auto& positional : args["positionals"]) {
          exec_args.positionals.push_back(positional.as<std::string>());
          arg_line.push_back(positional.as<std::string>());
        }
      }
    }
  // Assign args_line into the ExecutableArgs and append to vector
  exec_args.args_line = arg_line;
  executables.push_back(exec_args);
  // Debug: log parsed args for this executable
  std::stringstream ss;
  ss << "Parsed executable '" << exec_args.name << "'";
  for (const auto& a : arg_line) ss << " '" << a << "'";
  _logger.debug("%s", ss.str().c_str());
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
    size_t index) const {
  static const std::vector<std::string> empty;
  if (index >= executables.size()) return empty;
  return executables[index].args_line;
}
