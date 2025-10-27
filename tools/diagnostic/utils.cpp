#include "utils.hpp"

#include <array>
#include <cstdio>
#include <filesystem>
#include <stdexcept>
#ifdef __linux__
#include <sys/wait.h>
#endif

CommandResult run_command(const std::string& command) {
    std::array<char, 256> buffer{};
    std::string output;
    FILE* pipe = popen(command.c_str(), "r");
    if (!pipe) {
        return {-1, ""};
    }

    while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr) {
        output.append(buffer.data());
    }

    int status = pclose(pipe);
#ifdef __linux__
    if (WIFEXITED(status)) {
        status = WEXITSTATUS(status);
    }
#endif
    return {status, output};
}

std::string find_repo_root(int max_levels) {
    namespace fs = std::filesystem;
    fs::path current = fs::current_path();
    for (int level = 0; level <= max_levels; ++level) {
        if (fs::exists(current / ".root")) {
            return current.string();
        }
        if (!current.has_parent_path()) {
            break;
        }
        current = current.parent_path();
    }
    return {};
}
