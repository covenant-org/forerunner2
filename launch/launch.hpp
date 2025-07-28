#ifndef LAUNCH_HPP
#define LAUNCH_HPP

#include <filesystem>
#include <map>
#include <string>
#include <vector>
#include <thread>
#include <boost/process.hpp>
#include "logger.hpp"
#include <argparse/argparse.hpp>
#include "registry.hpp"

class launch {
protected:
    Core::ArgumentParser _args;
    Core::Logger _logger;

private:
    std::string _root_path;
    std::vector<std::string> _exclude_folders;
    Core::Registry _registry;

    static std::string find_root(const std::string& filename, int max_levels);
    static std::map<std::string, std::string> find_executable_files(
        const std::filesystem::path& dir,
        const std::vector<std::string>& exclude_folders);

public:
    std::map<std::string, std::string> executables;

    void set_log_level(Core::LogLevel level);
    launch(const std::vector<std::string>& exclude, const std::vector<std::string>& nodes);
    launch(const std::vector<std::string>& nodes);
    std::map<std::string, std::string> get_executables();
    static int run_executable(const std::string& executable_path);
    void run_executables(const std::vector<std::string>& nombres);
};

#endif // LAUNCH_HPP