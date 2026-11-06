// Bazel test entry point for the whole-body-controller regression benchmark.
// The controller stack is Python (MuJoCo bindings from the user site-packages),
// so this just runs tests/wbc_rbd_check.py and tests/wbc_benchmark.py with the
// system interpreter, the same way //:sim_gui launches run_interactive_sim.py.
#include <pwd.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstdlib>
#include <iostream>
#include <string>

static bool exists(const std::string& p) {
  struct stat st;
  return stat(p.c_str(), &st) == 0;
}

int main() {
  // Bazel points HOME at a sandbox dir; restore it so ~/.local site-packages resolve.
  if (const passwd* pw = getpwuid(getuid())) setenv("HOME", pw->pw_dir, 1);
  std::string py = "python3";
  if (const char* venv = std::getenv("VIRTUAL_ENV")) {
    if (exists(std::string(venv) + "/bin/python3")) py = std::string(venv) + "/bin/python3";
  }
  // Standalone dynamics vs MuJoCo first (catches a stale robot_model.h), then
  // the closed-loop scenarios.
  const std::string cmd = py + " tests/wbc_rbd_check.py && " + py + " tests/wbc_benchmark.py --backend cpp";
  std::cout << "[wbc_regression_test] " << cmd << std::endl;
  const int rc = std::system(cmd.c_str());
  return (rc == 0) ? 0 : 1;
}
