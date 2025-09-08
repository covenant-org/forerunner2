#ifndef NODES_YAML_PARSER_HPP
#define NODES_YAML_PARSER_HPP

#include <map>
#include <string>
#include <vector>

class NodesYamlParser {
 public:
  struct ExecutableArgs {
    std::string name;
    std::vector<std::string> flags;
    std::map<std::string, std::string> options;
    std::vector<std::string> positionals;
  };

  // Stores the formatted YAML string
  std::string yaml_str;
  // Stores formatted arguments per executable name.
  // Supports multiple entries per name (when the YAML contains executables
  // with the same `name`). Each inner vector is an argument line for one
  // instance of the executable.
  std::map<std::string, std::vector<std::vector<std::string>>> args_lines;
  // Stores the parsed executables
  std::vector<ExecutableArgs> executables;

  NodesYamlParser(const std::string& filename);
  // Devuelve el YAML leido y formateado
  const std::string& get_yaml() const;
  // Devuelve la línea de argumentos listo para CLI
  const std::vector<std::string>& get_args_line(
      const std::string& exec_name) const;

  // Returns all argument lines found for an executable name. Useful when
  // the YAML contains multiple executables with the same name.
  std::vector<std::vector<std::string>> get_args_lines(
      const std::string& exec_name) const;

  // Devuelve un vector con los nombres de los ejecutables
  std::vector<std::string> get_executables() const;
};

#endif  // NODES_YAML_PARSER_HPP
