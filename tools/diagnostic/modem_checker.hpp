#pragma once

#include "base_checker.hpp"

#include <string>

class ModemChecker : public BaseChecker {
public:
    ModemChecker(Core::Logger& logger, std::string interface_hint = "usb");
    DiagnosticResult run() override;

private:
    std::string interface_hint_;
};
