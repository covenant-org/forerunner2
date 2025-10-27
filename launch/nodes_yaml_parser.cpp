#include "nodes_yaml_parser.hpp"
#include <algorithm>
#include <cctype>
#include <sstream>
#include <yaml-cpp/yaml.h>

// Forward declarations in dependency order
void parse_key_value_or_flag(const std::string& input, NodesYamlParser::ExecutableArgs& exec_args);
void process_options_map(const YAML::Node& opts, NodesYamlParser::ExecutableArgs& exec_args);
void process_options_sequence(const YAML::Node& opts, NodesYamlParser::ExecutableArgs& exec_args);
void process_options_scalar(const YAML::Node& opts, NodesYamlParser::ExecutableArgs& exec_args);
void process_options_section(const YAML::Node& opts, NodesYamlParser::ExecutableArgs& exec_args);
void build_yaml_string(const NodesYamlParser::ExecutableArgs& exec_args, std::ostringstream& oss);

void process_args_section(const YAML::Node& args, NodesYamlParser::ExecutableArgs& exec_args, Core::Logger& logger) {
  // Process flags
  if (args["flags"]) {
    for (const auto& flag : args["flags"]) {
      std::string flag_str = flag.as<std::string>();
      exec_args.flags.push_back(flag_str);
      exec_args.args_line.push_back(flag_str);
    }
  }
  
  // Process options
  if (args["options"]) {
    process_options_section(args["options"], exec_args);
  }
  
  // Process positionals
  if (args["positionals"]) {
    for (const auto& positional : args["positionals"]) {
      std::string pos_str = positional.as<std::string>();
      exec_args.positionals.push_back(pos_str);
      exec_args.args_line.push_back(pos_str);
    }
  }
  
  // Debug log
  std::stringstream ss;
  ss << "Parsed executable '" << exec_args.name << "'";
  for (const auto& a : exec_args.args_line) ss << " '" << a << "'";
  logger.debug("%s", ss.str().c_str());
}

void process_options_section(const YAML::Node& opts, NodesYamlParser::ExecutableArgs& exec_args) {
  if (opts.IsMap()) {
    process_options_map(opts, exec_args);
  } else if (opts.IsSequence()) {
    process_options_sequence(opts, exec_args);
  } else if (opts.IsScalar()) {
    process_options_scalar(opts, exec_args);
  }
}

void process_options_map(const YAML::Node& opts, NodesYamlParser::ExecutableArgs& exec_args) {
  for (const auto& opt : opts) {
    std::string key = opt.first.as<std::string>();
    if (opt.second && !opt.second.IsNull()) {
      std::string val = opt.second.as<std::string>();
      exec_args.options[key] = val;
      exec_args.args_line.push_back(key);
      exec_args.args_line.push_back(val);
    } else {
      // Fallback: split key into "key value" or treat as flag
      parse_key_value_or_flag(key, exec_args);
    }
  }
}

void process_options_sequence(const YAML::Node& opts, NodesYamlParser::ExecutableArgs& exec_args) {
  for (const auto& n : opts) {
    std::string s = n.as<std::string>();
    parse_key_value_or_flag(s, exec_args);
  }
}

void process_options_scalar(const YAML::Node& opts, NodesYamlParser::ExecutableArgs& exec_args) {
  std::string s = opts.as<std::string>();
  parse_key_value_or_flag(s, exec_args);
}

void parse_key_value_or_flag(const std::string& input, NodesYamlParser::ExecutableArgs& exec_args) {
  std::istringstream iss(input);
  std::string key;
  if (!(iss >> key)) return;

  std::string remainder;
  std::getline(iss, remainder);

  if (!remainder.empty()) {
    remainder.erase(
        remainder.begin(),
        std::find_if(remainder.begin(), remainder.end(), [](unsigned char ch) {
          return !std::isspace(ch);
        }));

    exec_args.options[key] = remainder;
    exec_args.args_line.push_back(key);
    exec_args.args_line.push_back(remainder);
  } else {
    exec_args.flags.push_back(key);
    exec_args.args_line.push_back(key);
  }
}

void build_yaml_string(const NodesYamlParser::ExecutableArgs& exec_args, std::ostringstream& oss) {
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

NodesYamlParser::NodesYamlParser(const std::string& filename) {
  Core::Logger _logger(Core::LogLevel::INFO, "application.log", "nodes_yaml_parser");
  _logger.set_classname("nodes_yaml_parser");
  
  YAML::Node config = YAML::LoadFile(filename);
  std::ostringstream oss;
  
  for (const auto& exe : config["executables"]) {
    ExecutableArgs exec_args;
    exec_args.name = exe["name"].as<std::string>();
    
    if (exe["args"]) {
      process_args_section(exe["args"], exec_args, _logger);
    }
    
    executables.push_back(exec_args);
    
    // Build YAML string representation
    build_yaml_string(exec_args, oss);
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
