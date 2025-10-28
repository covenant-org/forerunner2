#include "modem_checker.hpp"

#include "utils.hpp"

#include <sstream>
#include <string>
#include <utility>

namespace {
constexpr const char* kCommand = "ip -br addr";

bool usb_interface_with_ip(const std::string& line, const std::string& hint) {
    std::istringstream line_stream(line);
    std::string iface;
    std::string state;
    if (!(line_stream >> iface >> state)) {
        return false;
    }

    if (iface.find(hint) == std::string::npos) {
        return false;
    }

    if (state == "DOWN") {
        return false;
    }

    std::string token;
    while (line_stream >> token) {
        if (token.find('/') != std::string::npos) {
            return true;
        }
    }
    return false;
}
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
        if (usb_interface_with_ip(line, interface_hint_)) {
            logger().debug("Matched 4G interface line: %s", line.c_str());
            return {name(), true, "4G modem connected with assigned IP"};
        }
    }

    logger().debug("No active interface containing '%s' with an IP was found", interface_hint_.c_str());
    return {name(), false, "No active 4G interface detected"};
}
