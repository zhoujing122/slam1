#include <filesystem>
#include <fstream>
#include <iostream>

#include "io/yaml_io.h"

int main() {
    namespace fs = std::filesystem;

    const fs::path yaml_path = fs::temp_directory_path() / "lightning_yaml_io_has_key_test.yaml";
    {
        std::ofstream fout(yaml_path);
        fout << "lidar_loc:\n";
        fout << "  init_with_fp: true\n";
        fout << "outer:\n";
        fout << "  middle:\n";
        fout << "    value: 3\n";
    }

    lightning::YAML_IO yaml(yaml_path.string());
    if (!yaml.HasKey("lidar_loc")) {
        std::cerr << "top-level key should exist\n";
        return 1;
    }
    if (!yaml.HasKey("lidar_loc", "init_with_fp")) {
        std::cerr << "nested key should exist\n";
        return 1;
    }
    if (yaml.HasKey("lidar_loc", "global_relocalization_filter_enable")) {
        std::cerr << "missing nested key should not exist\n";
        return 1;
    }
    if (yaml.HasKey("missing_group", "init_with_fp")) {
        std::cerr << "missing group should not expose nested keys\n";
        return 1;
    }
    if (!yaml.HasKey("outer", "middle", "value")) {
        std::cerr << "three-level key should exist\n";
        return 1;
    }
    if (yaml.HasKey("outer", "middle", "missing")) {
        std::cerr << "missing three-level key should not exist\n";
        return 1;
    }

    fs::remove(yaml_path);
    return 0;
}
