// Test helper, not a gtest: prints the tree the CM hands a controller, one leaf
// per line (`<dotted.key.path>\t<value>`).
//
//   controller_config_dump <controller.yaml> <config_key>
//
// test_shipped_catching_config.py runs it on the shipped catching configs and
// compares the lines with the Python loader's. The two loaders are separate
// implementations of one rule, so a Python tool reading a shipped profile only
// reads what the controller runs with while those lines agree.
#include "rtc_controller_manager/controller_config_loader.hpp"

#include <exception>
#include <iostream>

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "usage: controller_config_dump <controller.yaml> <config_key>\n";
    return 2;
  }
  try {
    const YAML::Node tree = rtc::LoadControllerConfig(argv[1], argv[2]);
    if (!tree.IsDefined() || !tree.IsMap()) {
      std::cerr << argv[1] << ": no map under '" << argv[2] << "'\n";
      return 1;
    }
    for (const auto& line : rtc::ControllerConfigLeafLines(tree)) {
      std::cout << line << '\n';
    }
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
  return 0;
}
