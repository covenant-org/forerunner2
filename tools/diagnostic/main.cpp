#include "base_checker.hpp"
#include "command_checker.hpp"
#include "modem_checker.hpp"

#include "argument_parser.hpp"
#include "utils.hpp"
#include "vertex.hpp"

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr const char* kDefaultZedCommand = "ZED_Diagnostic -c";
constexpr const char* kDefaultSerialUri = "serial:///dev/ttyTHS1:921600";
constexpr const char* kDefaultModemInterfaceHint = "usb";

std::string build_default_px4_command() {
    const std::string root = find_repo_root();

    if (root.empty()) {
        throw std::runtime_error("Could not locate .root marker; verify the repository layout");
    }

    std::filesystem::path binary = std::filesystem::path(root) / "build/mavlink/mavlink";

    std::string command = binary.string();
    if (command.find(' ') != std::string::npos) {
        command = "\"" + command + "\"";
    }
    return command + " --mavlink-uri \"" + std::string(kDefaultSerialUri) + "\"";
}
}

class DiagnosticApp : public Core::BaseVertex {
public:
    explicit DiagnosticApp(Core::ArgumentParser parser)
        : Core::BaseVertex(parser),
          exit_code_(0) {}

    void run() override {
        _logger.info("System diagnostics");

        std::vector<std::unique_ptr<BaseChecker>> checkers;
        const std::string px4_command = build_default_px4_command();
        const std::vector<std::string> px4_markers = {"MAVLink", "heartbeat"};
        checkers.emplace_back(std::make_unique<CommandChecker>(
            _logger,
            "PX4",
            px4_command,
            px4_markers,
            "Failed to execute MAVLink binary",
            "PX4 connected successfully",
            "No communication with PX4 detected",
            CommandChecker::MatchMode::Any,
            false));
        checkers.emplace_back(std::make_unique<ModemChecker>(_logger, std::string(kDefaultModemInterfaceHint)));
        const std::string zed_command = std::string(kDefaultZedCommand);
        const std::vector<std::string> zed_markers = {"running camera test diagnostic : ok"};
        checkers.emplace_back(std::make_unique<CommandChecker>(
            _logger,
            "ZED Camera",
            zed_command,
            zed_markers,
            "Failed to execute ZED diagnostic",
            "ZED camera detected successfully",
            "ZED camera not detected",
            CommandChecker::MatchMode::Any,
            false));

        bool all_ok = true;
        for (const auto& checker : checkers) {
            const DiagnosticResult result = checker->run();
            _logger.info("[%s] %s%s", result.name.c_str(), result.success ? "✅ " : "❌ ", result.message.c_str());
            all_ok = all_ok && result.success;
        }

        exit_code_ = all_ok ? 0 : 1;
    }

    [[nodiscard]] int exit_code() const { return exit_code_; }

private:
    int exit_code_;
};

int main(int argc, char** argv) {
    Core::BaseArgumentParser parser(argc, argv);

    try {
        auto app = std::make_shared<DiagnosticApp>(parser);
        app->run();
        return app->exit_code();
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
        return EXIT_FAILURE;
    }
}
