#include "modem_checker.hpp"

#include "utils.hpp"

#include <sstream>
#include <string>
#include <utility>

namespace {
constexpr const char* kCommand = "ip -br addr";
}

ModemChecker::ModemChecker(Core::Logger& logger, std::string interface_hint)
    : BaseChecker("4G Modem", logger), interface_hint_(std::move(interface_hint)) {}

DiagnosticResult ModemChecker::run() {
    logger().debug("Ejecutando comando: %s", kCommand);
    CommandResult result = run_command(kCommand);
    log_command_debug(kCommand, result);
    if (result.exit_code != 0) {
        return {name(), false, "No se pudo ejecutar `ip -br addr`"};
    }

    std::istringstream stream(result.output);
    std::string line;
    while (std::getline(stream, line)) {
        if (line.find(interface_hint_) == std::string::npos) {
            continue;
        }
        if (line.find("UNKNOWN") != std::string::npos) {
            continue;
        }
        if (line.find("DOWN") != std::string::npos) {
            continue;
        }
        if (line.find("inet") != std::string::npos) {
            logger().debug("Línea coincidente 4G: %s", line.c_str());
            return {name(), true, "Módem 4G conectado con IP asignada"};
        }
    }

    logger().debug("No se encontró una interfaz con IP que coincida con '%s'", interface_hint_.c_str());
    return {name(), false, "No se detectó interfaz 4G activa"};
}
