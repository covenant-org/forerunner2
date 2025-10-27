#ifndef LAUNCH_HPP
#define LAUNCH_HPP

#include "argument_parser.hpp"
#include "logger.hpp"
#include "nodes_yaml_parser.hpp"
#include <filesystem>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <yaml-cpp/yaml.h>


class Launch {
 protected:
  Core::Logger _logger;

 private:
  std::string _root_path;
  std::vector<std::string> _exclude_folders;
  std::string _log_level;
  inline static const std::vector<std::string> default_exclude_folders = {
      "vendor", ".git"};

  static std::map<std::string, std::string> find_executable_files(
      const std::filesystem::path& dir,
      const std::vector<std::string>& exclude_folders);
  
  static std::vector<std::vector<std::string>> build_args_from_yaml(
      argparse::ArgumentParser& parser, const NodesYamlParser& yaml_parser);
      
  static std::vector<std::string> build_registry_args(
      argparse::ArgumentParser& parser, const std::vector<std::string>& names);
  static std::string quote_argument(const std::string& arg);

 public:
  std::map<std::string, std::string> executables;
  

  Launch(argparse::ArgumentParser& parser,
         const std::vector<std::string>& exclude,
         const std::vector<std::string>& names,
         const std::vector<std::vector<std::string>>& args);
  Launch(argparse::ArgumentParser& parser,
         const std::vector<std::string>& names,
         const std::vector<std::vector<std::string>>& args);
  Launch(argparse::ArgumentParser& parser, const NodesYamlParser& yaml_parser);

  std::map<std::string, std::string> get_executables();
  int run_executable(const std::string& name,
                     const std::vector<std::string>& args);
  void run_executables(const std::vector<std::string>& names,
                       const std::vector<std::vector<std::string>>& arguments);
};

#endif  // LAUNCH_HPP
