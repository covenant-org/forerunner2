#include "nodes_yaml_parser.hpp"
#include <cctype>
#include <fstream>
#include <sstream>
#include <yaml-cpp/yaml.h>

// Forward declarations in dependency order
void parse_key_value_or_flag(const std::string& input,
                             NodesYamlParser::ExecutableArgs& exec_args);
void parse_key_value_or_flag_tokens(const std::vector<std::string>& tokens,
                                    NodesYamlParser::ExecutableArgs& exec_args);
void process_options_map(const YAML::Node& opts,
                         NodesYamlParser::ExecutableArgs& exec_args,
                         const std::string& source);
void process_options_sequence(const YAML::Node& opts,
                              NodesYamlParser::ExecutableArgs& exec_args,
                              const std::string& source);
void process_options_scalar(const YAML::Node& opts,
                            NodesYamlParser::ExecutableArgs& exec_args,
                            const std::string& source);
void process_options_section(const YAML::Node& opts,
                             NodesYamlParser::ExecutableArgs& exec_args,
                             const std::string& source);
void build_yaml_string(const NodesYamlParser::ExecutableArgs& exec_args,
                       std::ostringstream& oss);
bool node_is_quoted(const YAML::Node& node, const std::string& source);
std::vector<std::string> tokenize_node_arguments(const YAML::Node& node,
                                                 const std::string& source,
                                                 bool split_if_unquoted = true);
std::string join_tokens(const std::vector<std::string>& tokens);

void process_args_section(const YAML::Node& args,
                          NodesYamlParser::ExecutableArgs& exec_args,
                          Core::Logger& logger,
                          const std::string& source) {
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
    process_options_section(args["options"], exec_args, source);
  }
  
  // Process positionals
  if (args["positionals"]) {
    for (const auto& positional : args["positionals"]) {
      auto tokens = tokenize_node_arguments(positional, source);
      exec_args.positionals.insert(exec_args.positionals.end(), tokens.begin(),
                                   tokens.end());
      exec_args.args_line.insert(exec_args.args_line.end(), tokens.begin(),
                                 tokens.end());
    }
  }
  
  // Debug log
  std::stringstream ss;
  ss << "Parsed executable '" << exec_args.name << "'";
  for (const auto& a : exec_args.args_line) ss << " '" << a << "'";
  logger.debug("%s", ss.str().c_str());
}

void process_options_section(const YAML::Node& opts,
                             NodesYamlParser::ExecutableArgs& exec_args,
                             const std::string& source) {
  if (opts.IsMap()) {
    process_options_map(opts, exec_args, source);
  } else if (opts.IsSequence()) {
    process_options_sequence(opts, exec_args, source);
  } else if (opts.IsScalar()) {
    process_options_scalar(opts, exec_args, source);
  }
}

void process_options_map(const YAML::Node& opts,
                         NodesYamlParser::ExecutableArgs& exec_args,
                         const std::string& source) {
  for (const auto& opt : opts) {
    std::string key = opt.first.as<std::string>();
    if (opt.second && !opt.second.IsNull()) {
      auto value_tokens =
          tokenize_node_arguments(opt.second, source, /*split_if_unquoted=*/true);
      if (value_tokens.empty()) {
        exec_args.flags.push_back(key);
        exec_args.args_line.push_back(key);
        continue;
      }
      exec_args.options[key] = join_tokens(value_tokens);
      exec_args.args_line.push_back(key);
      exec_args.args_line.insert(exec_args.args_line.end(), value_tokens.begin(),
                                 value_tokens.end());
    } else {
      // Fallback: split key into "key value" or treat as flag
      parse_key_value_or_flag(key, exec_args);
    }
  }
}

void process_options_sequence(const YAML::Node& opts,
                              NodesYamlParser::ExecutableArgs& exec_args,
                              const std::string& source) {
  for (const auto& n : opts) {
    auto tokens = tokenize_node_arguments(n, source, /*split_if_unquoted=*/true);
    parse_key_value_or_flag_tokens(tokens, exec_args);
  }
}

void process_options_scalar(const YAML::Node& opts,
                            NodesYamlParser::ExecutableArgs& exec_args,
                            const std::string& source) {
  auto tokens = tokenize_node_arguments(opts, source, /*split_if_unquoted=*/true);
  parse_key_value_or_flag_tokens(tokens, exec_args);
}

void parse_key_value_or_flag(const std::string& input,
                             NodesYamlParser::ExecutableArgs& exec_args) {
  std::istringstream iss(input);
  std::vector<std::string> tokens;
  std::string token;
  while (iss >> token) tokens.push_back(token);
  parse_key_value_or_flag_tokens(tokens, exec_args);
}

void parse_key_value_or_flag_tokens(const std::vector<std::string>& tokens,
                                    NodesYamlParser::ExecutableArgs& exec_args) {
  if (tokens.empty()) return;

  const std::string& key = tokens.front();
  if (tokens.size() == 1) {
    exec_args.flags.push_back(key);
    exec_args.args_line.push_back(key);
    return;
  }

  std::vector<std::string> values(tokens.begin() + 1, tokens.end());
  exec_args.options[key] = join_tokens(values);
  exec_args.args_line.push_back(key);
  exec_args.args_line.insert(exec_args.args_line.end(), values.begin(),
                             values.end());
}

bool node_is_quoted(const YAML::Node& node, const std::string& source) {
  if (!node.IsScalar()) return false;

  YAML::Mark mark = node.Mark();
  if (mark.is_null() || mark.pos < 0) return false;

  std::size_t pos = static_cast<std::size_t>(mark.pos);
  if (pos >= source.size()) return false;

  // Skip indentation whitespace just in case the mark lands before the scalar
  while (pos < source.size() && std::isspace(static_cast<unsigned char>(source[pos]))) pos++;

  if (pos >= source.size()) return false;

  // Skip YAML tags such as !foo before checking the first significant character
  if (source[pos] == '!') {
    while (pos < source.size() && !std::isspace(static_cast<unsigned char>(source[pos]))) pos++;
    while (pos < source.size() && std::isspace(static_cast<unsigned char>(source[pos]))) pos++;
    if (pos >= source.size()) return false;
  }

  char c = source[pos];
  return (c == '\'' || c == '"' || c == '|' || c == '>');
}

std::vector<std::string> tokenize_node_arguments(const YAML::Node& node,
                                                 const std::string& source,
                                                 bool split_if_unquoted) {
  std::vector<std::string> tokens;
  if (!node) return tokens;

  if (node.IsSequence()) {
    for (const auto& child : node) {
      auto child_tokens =
          tokenize_node_arguments(child, source, split_if_unquoted);
      tokens.insert(tokens.end(), child_tokens.begin(), child_tokens.end());
    }
    return tokens;
  }

  if (node.IsScalar()) {
    std::string value = node.as<std::string>();
    bool quoted = node_is_quoted(node, source);

    if (!split_if_unquoted || quoted) {
      tokens.push_back(value);
    } else {
      std::istringstream iss(value);
      std::string part;
      while (iss >> part) tokens.push_back(part);
      if (tokens.empty() && !value.empty()) tokens.push_back(value);
    }
  }

  return tokens;
}

std::string join_tokens(const std::vector<std::string>& tokens) {
  std::ostringstream oss;
  for (std::size_t i = 0; i < tokens.size(); ++i) {
    if (i > 0) oss << ' ';
    oss << tokens[i];
  }
  return oss.str();
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
  Core::Logger _logger(Core::LogLevel::INFO, "application.log",
                       "nodes_yaml_parser");
  _logger.set_classname("nodes_yaml_parser");
  
  std::ifstream yaml_file(filename);
  if (!yaml_file.is_open()) {
    _logger.error("Unable to open YAML file: %s", filename.c_str());
    return;
  }

  std::ostringstream buffer;
  buffer << yaml_file.rdbuf();
  std::string yaml_source = buffer.str();

  YAML::Node config = YAML::Load(yaml_source);
  std::ostringstream oss;
  
  for (const auto& exe : config["executables"]) {
    ExecutableArgs exec_args;
    exec_args.name = exe["name"].as<std::string>();
    
    if (exe["args"]) {
      process_args_section(exe["args"], exec_args, _logger, yaml_source);
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
