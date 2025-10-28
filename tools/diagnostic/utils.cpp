#include "utils.hpp"

#include <array>
#include <cstdio>
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
