#include <iostream>
#include <cstdlib>
#include <string>
#include <sys/stat.h>

inline bool file_exists(const std::string& path) {
    struct stat buffer;
    return (stat(path.c_str(), &buffer) == 0);
}

int main(int argc, char** argv) {
    std::cout << "[Launcher] Launching Interactive MuJoCo GUI with Bazel C++ NMPC / WBC Controller...\n";
    std::string python_bin = "python3";
    if (const char* venv = std::getenv("VIRTUAL_ENV")) {
        std::string venv_py = std::string(venv) + "/bin/python3";
        if (file_exists(venv_py)) {
            python_bin = venv_py;
        }
    } else if (file_exists(".venv/bin/python3")) {
        python_bin = ".venv/bin/python3";
    } else if (file_exists("/home/manuel/bd_droid/.venv/bin/python3")) {
        python_bin = "/home/manuel/bd_droid/.venv/bin/python3";
    }

    std::string cmd = python_bin + " run_interactive_sim.py";
    for (int i = 1; i < argc; ++i) {
        cmd += " ";
        cmd += argv[i];
    }
    return std::system(cmd.c_str());
}
