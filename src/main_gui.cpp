#include <iostream>
#include <cstdlib>
#include <string>

int main(int argc, char** argv) {
    std::cout << "[Launcher] Launching Interactive MuJoCo GUI with Bazel C++ NMPC Controller...\n";
    std::string cmd = "python3 run_interactive_sim.py";
    for (int i = 1; i < argc; ++i) {
        cmd += " ";
        cmd += argv[i];
    }
    return std::system(cmd.c_str());
}
