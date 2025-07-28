#include "launch.hpp"


// Implementación de NodesYamlParser
NodesYamlParser::NodesYamlParser(const std::string& filename) {
    YAML::Node config = YAML::LoadFile(filename);
    for (const auto& exe : config["executables"]) {
        ExecutableArgs eargs;
        eargs.name = exe["name"].as<std::string>();
        const auto& args = exe["args"];
        if (args["flags"]) {
            for (const auto& flag : args["flags"]) {
                eargs.flags.push_back(flag.as<std::string>());
            }
        }
        if (args["options"]) {
            for (const auto& opt : args["options"]) {
                eargs.options[opt.first.as<std::string>()] = opt.second.as<std::string>();
            }
        }
        if (args["positionals"]) {
            for (const auto& pos : args["positionals"]) {
                eargs.positionals.push_back(pos.as<std::string>());
            }
        }
        executables.push_back(eargs);
    }
}

std::string NodesYamlParser::yaml() const {
    std::ostringstream oss;
    for (const auto& exe : executables) {
        oss << "Executable: " << exe.name << '\n';
        if (!exe.flags.empty()) {
            oss << "  Flags:";
            for (const auto& flag : exe.flags) {
                oss << " " << flag;
            }
            oss << '\n';
        }
        if (!exe.options.empty()) {
            oss << "  Options:" << '\n';
            for (const auto& opt : exe.options) {
                oss << "    " << opt.first << ": " << opt.second << '\n';
            }
        }
        if (!exe.positionals.empty()) {
            oss << "  Positionals:";
            for (const auto& pos : exe.positionals) {
                oss << " " << pos;
            }
            oss << '\n';
        }
    }
    return oss.str();
}

std::string launch::find_root(const std::string& filename, int max_levels) {
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

std::map<std::string, std::string> launch::find_executable_files(
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

void launch::set_log_level(Core::LogLevel level) {
    _logger.set_level(level);
}


launch::launch(int argc, char** argv, const std::vector<std::string>& exclude, const std::vector<std::string>& nodes)
    : _args(argc, argv), _exclude_folders(exclude) {
    // Aquí deberías agregar argumentos a _args si es necesario
    // _args.add_argument(...)
    // _args.parse(); // parse es protected, así que solo puedes usar la API pública

    // Configuración de logger (ajusta según tu API pública)
    // _logger.set_classname(...); // Si tienes acceso público
    // auto level = _args.get_argument<Core::LogLevel>("--log-level");
    // _logger.set_level(level);

    _root_path = find_root(".root", 10);
    if (_root_path.empty()) {
        std::cerr << "Root path not found." << std::endl;
        return;
    }
    executables = find_executable_files(_root_path, _exclude_folders);
    if (!nodes.empty()) {
        run_executables(nodes);
    }
}

launch::launch(int argc, char** argv, const std::vector<std::string>& nodes)
    : launch(argc, argv, std::vector<std::string>{"vendor", ".git"}, nodes) {}

std::map<std::string, std::string> launch::get_executables() {
    return executables;
}

int launch::run_executable(const std::string& executable_path) {
    int result = std::system(executable_path.c_str());
    return result;
}

void launch::run_executables(const std::vector<std::string>& nombres) {
    std::vector<std::thread> threads;
    for (const auto& nombre : nombres) {
        auto it = executables.find(nombre);
        if (it != executables.end()) {
            threads.emplace_back([nombre, path = it->second]() {
                try {
                    boost::process::child c(path);
                    c.wait();
                    std::cout << nombre << " terminó con código: " << c.exit_code() << std::endl;
                } catch (const std::exception& e) {
                    std::cerr << "Error ejecutando " << nombre << ": " << e.what() << std::endl;
                }
            });
        } else {
            std::cout << "No se encontró ejecutable: " << nombre << std::endl;
        }
    }
    for (auto& t : threads) t.join();
}

int main(int argc, char** argv) {
    launch launch_instance(argc, argv, std::vector<std::string>{"executable_test", "executable_test2"});
    return 0;
}