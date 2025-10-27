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
    logger().debug("Running command: %s", kCommand);
    CommandResult result = run_command(kCommand);
    log_command_debug(kCommand, result);
    if (result.exit_code != 0) {
        return {name(), false, "Unable to execute `ip -br addr`"};
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
            logger().debug("Matched 4G interface line: %s", line.c_str());
            return {name(), true, "4G modem connected with assigned IP"};
        }
    }

    logger().debug("No active interface containing '%s' with an IP was found", interface_hint_.c_str());
    return {name(), false, "No active 4G interface detected"};
}
