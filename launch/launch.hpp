#ifndef LAUNCH_HPP
#define LAUNCH_HPP

#include <filesystem>
#include <map>
#include <string>
#include <vector>
#include <thread>
#include <boost/process.hpp>
#include <yaml-cpp/yaml.h>
#include <sstream>
#include <iostream>

#include "logger.hpp"
#include "argument_parser.hpp"
#include "nodes_yaml_parser.hpp"


class Launch {
protected:
    Core::Logger _logger;

private:
    std::string _root_path;
    std::vector<std::string> _exclude_folders;
    inline static const std::vector<std::string> default_exclude_folders = {"vendor", ".git"};

    std::string find_root(const std::string& filename, int max_levels);
    static std::map<std::string, std::string> find_executable_files(
        const std::filesystem::path& dir,
        const std::vector<std::string>& exclude_folders);

public:
    std::map<std::string, std::string> executables;

    void set_log_level(Core::LogLevel level);

    Launch(argparse::ArgumentParser& parser, const std::vector<std::string>& exclude, const std::vector<std::string>& names, const std::vector<std::vector<std::string>>& args);
    Launch(argparse::ArgumentParser& parser, const std::vector<std::string>& names, const std::vector<std::vector<std::string>>& args);
    Launch(argparse::ArgumentParser& parser, const NodesYamlParser& yaml_parser);

    std::map<std::string, std::string> get_executables();
    int run_executable(const std::string& name, const std::vector<std::string>& args);
    void run_executables(const std::vector<std::string>& names, const std::vector<std::vector<std::string>>& arguments);
};

#endif // LAUNCH_HPP