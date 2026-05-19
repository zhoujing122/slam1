#include <cmath>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>

#include "core/maps/tiled_map.h"

namespace {

bool Near(double a, double b) { return std::fabs(a - b) < 1e-6; }

bool ContainsXY(const std::vector<lightning::Vec3d>& pts, double x, double y) {
    for (const auto& p : pts) {
        if (Near(p.x(), x) && Near(p.y(), y)) {
            return true;
        }
    }
    return false;
}

}  // namespace

int main() {
    namespace fs = std::filesystem;
    using namespace lightning;

    const fs::path map_dir = fs::temp_directory_path() / "lightning_reloc_candidate_test";
    fs::remove_all(map_dir);
    fs::create_directories(map_dir);

    CloudPtr chunk_cloud(new PointCloudType);
    for (double x : {0.0, 5.0, 10.0}) {
        for (double y : {0.0, 5.0, 10.0}) {
            PointType pt;
            pt.x = static_cast<float>(x);
            pt.y = static_cast<float>(y);
            pt.z = 1.0f;
            chunk_cloud->push_back(pt);
        }
    }

    const std::string pcd_path = (map_dir / "0.pcd").string();
    pcl::io::savePCDFileBinary(pcd_path, *chunk_cloud);

    std::ofstream index(map_dir / "index.txt");
    index << "0 0 0\n";
    index << "0 0 0 " << pcd_path << "\n";
    index << "# functional points\n";
    index.close();

    TiledMap::Options options;
    options.map_path_ = map_dir.string();
    options.chunk_size_ = 20.0f;
    options.inv_chunk_size_ = 1.0f / options.chunk_size_;

    TiledMap map(options);
    if (!map.LoadMapIndex()) {
        std::cerr << "failed to load test map index\n";
        return 1;
    }

    const auto sampled = map.GetRelocalizationCandidatePositions(5.0);
    if (sampled.size() != 9) {
        std::cerr << "expected 9 sampled candidates, got " << sampled.size() << "\n";
        return 1;
    }

    if (!ContainsXY(sampled, 0.0, 0.0) || !ContainsXY(sampled, 5.0, 5.0) ||
        !ContainsXY(sampled, 10.0, 10.0)) {
        std::cerr << "sampled candidates do not cover expected grid points\n";
        return 1;
    }

    const auto fallback = map.GetRelocalizationCandidatePositions(0.0);
    if (fallback.size() != 1 || !ContainsXY(fallback, 0.0, 0.0)) {
        std::cerr << "invalid sample step should fall back to chunk center candidates\n";
        return 1;
    }

    fs::remove_all(map_dir);
    return 0;
}
