#ifndef NODES_YAML_PARSER_HPP
#define NODES_YAML_PARSER_HPP

#include <map>
#include <string>
#include <vector>
#include "logger.hpp"

class NodesYamlParser {
 protected:
  Core::Logger _logger;

 public:
  struct ExecutableArgs {
    std::string name;
    std::vector<std::string> flags;
    std::map<std::string, std::string> options;
    std::vector<std::string> positionals;
    std::vector<std::string> args_line;
  };

  // Stores the formatted YAML string
  std::string yaml_str;
  // Stores the parsed executables
  std::vector<ExecutableArgs> executables;

  NodesYamlParser(const std::string& filename);
  // Devuelve el YAML leido y formateado
  const std::string& get_yaml() const;

  // Returns the argument line for the executable at the given YAML index
  // (preserves duplicates and YAML ordering).
  const std::vector<std::string>& get_args_line(size_t index) const;

  // Returns all argument lines found for an executable name. Useful when
  // the YAML contains multiple executables with the same name.
  std::vector<std::vector<std::string>> get_args_lines(
      const std::string& exec_name) const;

  // Devuelve un vector con los nombres de los ejecutables
  std::vector<std::string> get_executables() const;
};

#endif  // NODES_YAML_PARSER_HPP
