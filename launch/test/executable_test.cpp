#include <iostream>
#include <string>

int main(int argc, char** argv) {
    std::cout << "[executable_test] Recibidos " << argc-1 << " argumentos:" << std::endl;
    for (int i = 1; i < argc; ++i) {
        std::cout << "  arg[" << i << "]: " << argv[i] << std::endl;
    }

    // Simulación de uso de flags y options
    bool verbose = false, log = false;
    std::string input, threshold, mode;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--verbose") verbose = true;
        if (arg == "--log") log = true;
        if (arg == "--input" && i+1 < argc) input = argv[++i];
        if (arg == "--threshold" && i+1 < argc) threshold = argv[++i];
        if (arg == "--mode" && i+1 < argc) mode = argv[++i];
    }
    std::cout << "Flags: verbose=" << verbose << ", log=" << log << std::endl;
    std::cout << "Options: input='" << input << "', threshold='" << threshold << "', mode='" << mode << "'" << std::endl;
    return 0;
}