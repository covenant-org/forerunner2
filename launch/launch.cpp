#include <iostream>
#include <filesystem>
#include <vector>
#include <string>
#include <map>
#include <boost/process.hpp>
#include <thread>


class launch {

    private:
    // Variables miembro de ejemplo
    std::string _root_path;
    std::vector<std::string> _exclude_folders;

    static std::string find_root(const std::string& filename, int max_levels) {
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

    static std::map<std::string, std::string> find_executable_files(
        const std::filesystem::path& dir,
        const std::vector<std::string>& exclude_folders = {"vendor", ".git"}
    ) {
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

    public:
    // Constructor parametrizado que realiza la lógica principal
    std::map<std::string, std::string> executables;

    launch(
        const std::vector<std::string>& exclude = {"vendor", ".git"},
        const std::vector<std::string>& nodes = {}
    ) : _exclude_folders(exclude) {
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

    // Sobrecarga: solo recibe nodos
    launch(const std::vector<std::string>& nodes)
        : launch({"vendor", ".git"}, nodes) {}

    std::map<std::string, std::string> get_executables() {
        return executables;
    }

    static int run_executable(const std::string& executable_path) {
        int result = std::system(executable_path.c_str());
        return result;
    }

    void run_executables(const std::vector<std::string>& nombres) {
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
};

int main() {
    launch launch_instance(std::vector<std::string>{"executable_test", "executable_test2"});
    return 0;
}