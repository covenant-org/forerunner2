
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

// Parser de YAML para ejecutables
class NodesYamlParser {
public:
    struct ExecutableArgs {
        std::string name;
        std::vector<std::string> flags;
        std::map<std::string, std::string> options;
        std::vector<std::string> positionals;
    };

    std::vector<ExecutableArgs> executables;

    NodesYamlParser(const std::string& filename);
    std::string yaml() const;
};




class launch {
protected:
    Core::ArgumentParser _args;
    Core::Logger _logger;

private:
    std::string _root_path;
    std::vector<std::string> _exclude_folders;

    static std::string find_root(const std::string& filename, int max_levels);
    static std::map<std::string, std::string> find_executable_files(
        const std::filesystem::path& dir,
        const std::vector<std::string>& exclude_folders);

public:
    std::map<std::string, std::string> executables;

    void set_log_level(Core::LogLevel level);
    launch(int argc, char** argv, const std::vector<std::string>& exclude, const std::vector<std::string>& nodes);
    launch(int argc, char** argv, const std::vector<std::string>& nodes);
    std::map<std::string, std::string> get_executables();
    static int run_executable(const std::string& executable_path);
    void run_executables(const std::vector<std::string>& nombres);
};

#endif // LAUNCH_HPP