#pragma once

#include <string>

struct CommandResult {
    int exit_code;
    std::string output;
};

CommandResult run_command(const std::string& command);
