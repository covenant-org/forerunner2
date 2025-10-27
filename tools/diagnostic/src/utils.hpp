#pragma once

#include <string>

struct CommandResult {
    int exit_code;
    std::string output;
};

CommandResult run_command(const std::string& command);
std::string find_repo_root(int max_levels = 10);
