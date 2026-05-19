#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

int main() {
    const std::filesystem::path source_path =
        std::filesystem::path(LIGHTNING_SOURCE_DIR) / "src/core/localization/lidar_loc/lidar_loc.cc";

    std::ifstream fin(source_path);
    if (!fin) {
        std::cerr << "failed to open " << source_path << "\n";
        return 1;
    }

    std::string content((std::istreambuf_iterator<char>(fin)), std::istreambuf_iterator<char>());
    if (content.find("std::clamp(step, 4, 360)") == std::string::npos) {
        std::cerr << "YawSearch step clamp is missing\n";
        return 1;
    }
    if (content.find("static_cast<int>(lidar_loc::grid_search_angle_step)") == std::string::npos) {
        std::cerr << "YawSearch should still derive default step from yaml value\n";
        return 1;
    }
    return 0;
}
