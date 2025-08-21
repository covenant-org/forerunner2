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
  std::vector<std::string> all_names;
  std::vector<std::vector<std::string>> all_args;
  std::string _root_path;
  std::vector<std::string> _exclude_folders;
  double _delay_seconds = 0.0;
  std::string _log_level = "info";
  inline static const std::vector<std::string> default_exclude_folders = {
      "vendor", ".git"};

  static std::map<std::string, std::string> find_executable_files(
      const std::filesystem::path& dir,
      const std::vector<std::string>& exclude_folders);

 public:
  std::map<std::string, std::string> executables;

  Launch(argparse::ArgumentParser& parser,
         const std::vector<std::string>& exclude,
         const std::vector<std::string>& names,
         const std::vector<std::vector<std::string>>& args,
         double delay_seconds = 0.0);
  Launch(argparse::ArgumentParser& parser,
         const std::vector<std::string>& names,
         const std::vector<std::vector<std::string>>& args,
         double delay_seconds = 0.0);
  Launch(argparse::ArgumentParser& parser, const NodesYamlParser& yaml_parser, double delay_seconds = 0.0);

  std::map<std::string, std::string> get_executables();

  static Core::LogLevel string_to_loglevel(const std::string& level);

  int run_executable(const std::string& name,
                     const std::vector<std::string>& args);
  void run_executables(const std::vector<std::string>& names,
                       const std::vector<std::vector<std::string>>& arguments);
  void run_executables();
};

#endif  // LAUNCH_HPP
