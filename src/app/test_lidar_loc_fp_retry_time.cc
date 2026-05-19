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
    if (content.find("1e-6 * static_cast<double>(input->header.stamp)") != std::string::npos) {
        std::cerr << "legacy fp retry timestamp conversion still present\n";
        return 1;
    }

    const std::string helper = "LidarInputEndTime(input)";
    std::size_t pos = 0;
    int helper_count = 0;
    while ((pos = content.find(helper, pos)) != std::string::npos) {
        ++helper_count;
        pos += helper.size();
    }

    if (helper_count < 3) {
        std::cerr << "expected helper to be used in Align and InitWithFP paths\n";
        return 1;
    }

    return 0;
}
