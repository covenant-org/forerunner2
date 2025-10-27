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
#include <string>
#include <vector>

namespace {
constexpr const char* kDefaultZedCommand = "ZED_Diagnostic -c";
constexpr const char* kDefaultSerialUri = "serial:///dev/ttyTHS1:921600";

std::filesystem::path first_existing_path(const std::vector<std::filesystem::path>& candidates) {
    for (const auto& path : candidates) {
        if (std::filesystem::exists(path)) {
            return path;
        }
    }
    return candidates.empty() ? std::filesystem::path{} : candidates.front();
}

std::string build_default_px4_command() {
    const std::string root = find_repo_root();

    auto make_path = [&](const std::string& relative) {
        return root.empty() ? std::filesystem::path(relative)
                            : std::filesystem::path(root) / relative;
    };

    const std::vector<std::filesystem::path> candidates = {
        make_path("build/mavlink/mavlink"),
        make_path("build/mavlink/mavlink/mavlink"),
        make_path("mavlink/mavlink"),
        make_path("mavlink/mavlink/mavlink")
    };

    std::filesystem::path binary = first_existing_path(candidates);
    std::string command = binary.string();
    if (command.empty()) {
        command = "build/mavlink/mavlink";
    }
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
          px4_command_(this->get_argument<std::string>("--px4-command")),
          modem_hint_(this->get_argument<std::string>("--modem-interface")),
          zed_command_(this->get_argument<std::string>("--zed-command")),
          exit_code_(0) {}

    void run() override {
        _logger.info("=== Diagnóstico del sistema ===");

        std::vector<std::unique_ptr<BaseChecker>> checkers;
        const std::string px4_command = px4_command_.empty() ? build_default_px4_command() : px4_command_;
        const std::vector<std::string> px4_markers = {"MAVLink", "heartbeat"};
        checkers.emplace_back(std::make_unique<CommandChecker>(
            _logger,
            "PX4",
            px4_command,
            px4_markers,
            "No se pudo ejecutar el binario MAVLink",
            "PX4 conectada correctamente",
            "No se detecta comunicación con la PX4",
            CommandChecker::MatchMode::Any,
            false));
        checkers.emplace_back(std::make_unique<ModemChecker>(_logger, modem_hint_));
        const std::string zed_command = zed_command_.empty() ? std::string(kDefaultZedCommand) : zed_command_;
        const std::vector<std::string> zed_markers = {"running camera test diagnostic : ok"};
        checkers.emplace_back(std::make_unique<CommandChecker>(
            _logger,
            "ZED Camera",
            zed_command,
            zed_markers,
            "No se pudo ejecutar el diagnóstico de la ZED",
            "ZED detectada correctamente",
            "No se detectó la cámara ZED",
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
    std::string px4_command_;
    std::string modem_hint_;
    std::string zed_command_;
    int exit_code_;
};

int main(int argc, char** argv) {
    Core::BaseArgumentParser parser(argc, argv);

    parser.add_argument("--px4-command")
        .default_value(std::string{})
        .help("Comando completo para verificar la PX4 (sobrescribe el valor por defecto)");
    parser.add_argument("--modem-interface")
        .default_value(std::string{"usb"})
        .help("Fragmento de nombre de la interfaz 4G a verificar");
    parser.add_argument("--zed-command")
        .default_value(std::string{kDefaultZedCommand})
        .help("Comando a ejecutar para diagnosticar la cámara ZED");

    try {
        auto app = std::make_shared<DiagnosticApp>(parser);
        app->run();
        return app->exit_code();
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
        return EXIT_FAILURE;
    }
}
