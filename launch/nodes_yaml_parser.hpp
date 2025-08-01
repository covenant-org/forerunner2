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

  // Almacena el string YAML ya formateado
  std::string yaml_str;
  // Almacena los argumentos ya formateados por ejecutable
  std::map<std::string, std::vector<std::string>> args_lines;
  // Almacena los ejecutables parseados
  std::vector<ExecutableArgs> executables;

  NodesYamlParser(const std::string& filename);
  // Devuelve el YAML leido y formateado
  const std::string& get_yaml() const;
  // Devuelve la línea de argumentos listo para CLI
  const std::vector<std::string>& get_args_line(
      const std::string& exec_name) const;

  // Devuelve un vector con los nombres de los ejecutables
  std::vector<std::string> get_executables() const;
};

#endif  // NODES_YAML_PARSER_HPP
