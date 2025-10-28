#include "diagnostic_app.hpp"

#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>

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
