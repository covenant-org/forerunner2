#pragma once

#include "core/vertex.hpp"

class DiagnosticApp : public Core::BaseVertex {
public:
    explicit DiagnosticApp(Core::ArgumentParser parser);

    void run() override;

    [[nodiscard]] int exit_code() const;

private:
    int exit_code_;
};
