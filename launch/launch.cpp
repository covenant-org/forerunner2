#include "launch.hpp"


std::string Launch::find_root(const std::string& filename, int max_levels) {
    std::filesystem::path current = std::filesystem::current_path();
    for (int i = 0; i <= max_levels; ++i) {
        for (const auto& entry : std::filesystem::directory_iterator(current)) {
            if (entry.is_regular_file() && entry.path().filename() == filename) {
                return entry.path().parent_path().string();
            }
        }
        if (current.has_parent_path()) {
            current = current.parent_path();
        } else {
            break;
        }
    }
    std::cout << "Root not found" << std::endl;
    return "";
}

std::map<std::string, std::string> Launch::find_executable_files(
    const std::filesystem::path& dir,
    const std::vector<std::string>& exclude_folders) {
    std::map<std::string, std::string> exe_map;
    for (const auto& entry : std::filesystem::recursive_directory_iterator(dir)) {
        if (entry.is_regular_file()) {
            // Excluir carpetas
            if (!exclude_folders.empty()) {
                bool in_excluded = false;
                auto rel = std::filesystem::relative(entry.path(), dir);
                for (const auto& folder : exclude_folders) {
                    if (rel.string().find(folder + "/") == 0 || rel.string() == folder) {
                        in_excluded = true;
                        break;
                    }
                }
                if (in_excluded) continue;
            }
            // Filtrar archivos con extensión
            if (entry.path().has_extension()) continue;

            auto perms = entry.status().permissions();
            if ((perms & std::filesystem::perms::owner_exec) != std::filesystem::perms::none ||
                (perms & std::filesystem::perms::group_exec) != std::filesystem::perms::none ||
                (perms & std::filesystem::perms::others_exec) != std::filesystem::perms::none) {
                std::string exe_name = entry.path().filename().string();
                exe_map[exe_name] = entry.path().string();
            }
        }
    }
    return exe_map;
}

void Launch::set_log_level(Core::LogLevel level) {
    _logger.set_level(level);
}



Launch::Launch(int argc, char** argv, const std::vector<std::string>& exclude, const std::vector<std::string>& names, const std::vector<std::vector<std::string>>& args)
    : _args(argc, argv), _exclude_folders(exclude) {
    registry_port_ = 0;
    registry_threads_ = 0;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--registry-port" && i + 1 < argc) {
            registry_port_ = std::stoi(argv[i + 1]);
        }
        if (arg == "--registry-threads" && i + 1 < argc) {
            registry_threads_ = std::stoi(argv[i + 1]);
        }
    }

    _root_path = find_root(".root", 10);
    if (_root_path.empty()) {
        std::cerr << "Root path not found." << std::endl;
        return;
    }
    executables = find_executable_files(_root_path, _exclude_folders);

    std::vector<std::string> all_names = names;
    std::vector<std::vector<std::string>> all_args = args;
    std::vector<std::string> registry_args;
    if (registry_port_ != 0 && registry_threads_ != 0) {
        registry_args = {"--port", std::to_string(registry_port_), "--threads", std::to_string(registry_threads_)};
    }
    else { registry_args = {}; }
    all_names.insert(all_names.begin(), "registry");
    all_args.insert(all_args.begin(), registry_args);
    if (!all_names.empty()) {
        run_executables(all_names, all_args);
    }
}

Launch::Launch(int argc, char** argv, const std::vector<std::string>& names, const std::vector<std::vector<std::string>>& args)
    : Launch(argc, argv, default_exclude_folders, names, args) {}

// Constructor que recibe un NodesYamlParser
Launch::Launch(int argc, char** argv, const NodesYamlParser& parser)
    : Launch(argc, argv, default_exclude_folders,
        parser.get_executables(),
        [&parser]() {
            std::vector<std::vector<std::string>> args;
            for (const auto& name : parser.get_executables()) {
                args.push_back(parser.get_args_line(name));
            }
            return args;
        }()) {}



std::map<std::string, std::string> Launch::get_executables() {
    return executables;
}

int Launch::run_executable(const std::string& name, const std::vector<std::string>& args) {
    auto it = executables.find(name);
    if (it == executables.end()) {
        std::cout << "Executable not found: " << name << std::endl;
        return -1;
    }
    try {
        if (args.empty()) {
            boost::process::child c(it->second);
            c.wait();
            std::cout << name << " finished with code: " << c.exit_code() << std::endl;
            return c.exit_code();
        } else {
            boost::process::child c(it->second, boost::process::args(args));
            c.wait();
            std::cout << name << " finished with code: " << c.exit_code() << std::endl;
            return c.exit_code();
        }
    } catch (const std::exception& e) {
        std::cerr << "Error running " << name << ": " << e.what() << std::endl;
        return -1;
    }
}

void Launch::run_executables(const std::vector<std::string>& names, const std::vector<std::vector<std::string>>& arguments) {
    std::vector<std::thread> threads;
    for (size_t i = 0; i < names.size(); ++i) {
        std::vector<std::string> args;
        if (i < arguments.size()) {
            args = arguments[i];
        }
        threads.emplace_back([this, name = names[i], args]() {
            run_executable(name, args);
            if (name == "registry") {
                std::this_thread::sleep_for(std::chrono::seconds(2));
            }
        });
    }
    for (auto& t : threads) {
        if (t.joinable()) t.join();
    }
}

void print_help() {
    std::cout << "\nUsage: launch [options]\n"
              << "  --yaml-path <path>         Path to the YAML configuration file (required)\n"
              << "  --registry-port <port>     Registry port (optional, requires --registry-threads)\n"
              << "  --registry-threads <n>     Number of threads for the registry (optional, requires --registry-port)\n"
              << "  --help, -h                 Show this help message\n"
              << "\nNotes:\n  - --yaml-path is required.\n  - If you use --registry-port you must also use --registry-threads, and vice versa.\n";
}

int main(int argc, char** argv) {
    std::string yaml_path;
    int registry_port = 0;
    int registry_threads = 0;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            print_help();
            return 0;
        }
        if (arg == "--yaml-path" && i + 1 < argc) {
            yaml_path = argv[i + 1];
        }
        if (arg == "--registry-port" && i + 1 < argc) {
            registry_port = std::stoi(argv[i + 1]);
        }
        if (arg == "--registry-threads" && i + 1 < argc) {
            registry_threads = std::stoi(argv[i + 1]);
        }
    }
    // Validación de argumentos obligatorios y dependientes
    if (yaml_path.empty() ||
        ((registry_port != 0 && registry_threads == 0) || (registry_threads != 0 && registry_port == 0))) {
        print_help();
        return 1;
    }
    NodesYamlParser parser(yaml_path);
    Launch launch_instance(argc, argv, parser);
    return 0;
}

// int main() {
//     int argc = 1;
//     char arg0[] = "dummy";
//     char* argv_fake[] = {arg0, nullptr};
//     Launch l(argc, argv_fake, std::vector<std::string>{}, std::vector<std::vector<std::string>>{});
//     const auto& exes = l.get_executables();
//     std::cout << "Executables encontrados:" << std::endl;
//     for (const auto& exe : exes) {
//         std::cout << "Nombre: " << exe.first << " | Ruta: " << exe.second << std::endl;
//     }
//     return 0;
// }


