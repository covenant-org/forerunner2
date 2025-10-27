#pragma once

#include "logger.hpp"

#include <string>
#include <utility>

struct DiagnosticResult {
    std::string name;
    bool success;
    std::string message;
};

struct CommandResult;

class BaseChecker {
public:
    BaseChecker(std::string name, Core::Logger& logger);
    virtual ~BaseChecker() = default;

    virtual DiagnosticResult run() = 0;

protected:
    const std::string& name() const { return name_; }
    Core::Logger& logger() { return logger_; }
    Core::Logger& logger() const { return logger_; }
    void log_command_debug(const std::string& command, const CommandResult& result) const;

private:
    std::string name_;
    Core::Logger& logger_;
};

#include "utils.hpp"

inline BaseChecker::BaseChecker(std::string name, Core::Logger& logger)
    : name_(std::move(name)), logger_(logger) {}

inline void BaseChecker::log_command_debug(const std::string& command, const CommandResult& result) const {
    logger_.debug("Comando '%s' finalizó con código %d", command.c_str(), result.exit_code);
    if (!result.output.empty()) {
        logger_.debug("Salida de %s:\n%s", name_.c_str(), result.output.c_str());
    }
}
