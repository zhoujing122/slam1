#include <cmath>
#include <cstdlib>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

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

void AddPoint(lightning::CloudPtr cloud, double x, double y, double z) {
    lightning::PointType pt;
    pt.x = static_cast<float>(x);
    pt.y = static_cast<float>(y);
    pt.z = static_cast<float>(z);
    cloud->push_back(pt);
}

bool IsSortedXY(const std::vector<lightning::Vec3d>& pts) {
    for (size_t i = 1; i < pts.size(); ++i) {
        if (pts[i - 1].x() > pts[i].x()) {
            return false;
        }
        if (Near(pts[i - 1].x(), pts[i].x()) && pts[i - 1].y() > pts[i].y()) {
            return false;
        }
    }
    return true;
}

bool WriteMap(const std::filesystem::path& map_dir, const lightning::CloudPtr& cloud) {
    namespace fs = std::filesystem;
    fs::remove_all(map_dir);
    fs::create_directories(map_dir);

    const std::string pcd_path = (map_dir / "0.pcd").string();
    if (pcl::io::savePCDFileBinary(pcd_path, *cloud) != 0) {
        return false;
    }

    std::ofstream index(map_dir / "index.txt");
    index << "0 0 0\n";
    index << "0 0 0 " << pcd_path << "\n";
    index << "# functional points\n";
    return static_cast<bool>(index);
}

std::unique_ptr<lightning::TiledMap> LoadMap(const std::filesystem::path& map_dir) {
    lightning::TiledMap::Options options;
    options.map_path_ = map_dir.string();
    options.chunk_size_ = 20.0f;
    options.inv_chunk_size_ = 1.0f / options.chunk_size_;

    auto map = std::make_unique<lightning::TiledMap>(options);
    if (!map->LoadMapIndex()) {
        std::cerr << "failed to load test map index\n";
        std::exit(1);
    }
    return map;
}

lightning::TiledMap::RelocCandidateFilterOptions TestFilterOptions(double sample_step) {
    lightning::TiledMap::RelocCandidateFilterOptions opt;
    opt.sample_step = sample_step;
    opt.filter_enable = true;
    opt.min_chunk_points = 1;
    opt.grid_resolution = 0.1;
    opt.obstacle_z_min = 0.15;
    opt.obstacle_z_max = 1.5;
    opt.clear_radius = 0.3;
    opt.support_radius = 0.75;
    opt.min_support_cells = 1;
    return opt;
}

}  // namespace

int main() {
    namespace fs = std::filesystem;
    using namespace lightning;

    const fs::path map_dir = fs::temp_directory_path() / "lightning_reloc_candidate_test";

    CloudPtr chunk_cloud(new PointCloudType);
    for (double x : {0.0, 5.0, 10.0}) {
        for (double y : {0.0, 5.0, 10.0}) {
            AddPoint(chunk_cloud, x, y, 1.0);
        }
    }
    AddPoint(chunk_cloud, 10.0, 10.0, 1.0);

    if (!WriteMap(map_dir, chunk_cloud)) {
        std::cerr << "failed to write sample map\n";
        return 1;
    }

    auto map = LoadMap(map_dir);
    const auto sampled = map->GetRelocalizationCandidatePositions(5.0);
    if (sampled.size() != 9) {
        std::cerr << "expected 9 sampled candidates, got " << sampled.size() << "\n";
        return 1;
    }

    if (!ContainsXY(sampled, 0.0, 0.0) || !ContainsXY(sampled, 5.0, 5.0) ||
        !ContainsXY(sampled, 10.0, 10.0)) {
        std::cerr << "sampled candidates do not cover expected grid points\n";
        return 1;
    }

    const auto invalid = map->GetRelocalizationCandidatePositions(0.0);
    if (!invalid.empty()) {
        std::cerr << "invalid sample step should not fall back to chunk center candidates\n";
        return 1;
    }

    if (!IsSortedXY(sampled)) {
        std::cerr << "sampled candidates should be sorted by x/y for stable auto_* ids\n";
        return 1;
    }

    const fs::path obstacle_dir = fs::temp_directory_path() / "lightning_reloc_obstacle_test";
    CloudPtr wall_cloud(new PointCloudType);
    for (double y : {-0.5, 0.0, 0.5}) {
        AddPoint(wall_cloud, 0.0, y, 1.0);
    }
    if (!WriteMap(obstacle_dir, wall_cloud)) {
        std::cerr << "failed to write obstacle map\n";
        return 1;
    }

    auto obstacle_map = LoadMap(obstacle_dir);
    auto obstacle_opt = TestFilterOptions(0.5);
    const auto obstacle_filtered = obstacle_map->GetRelocalizationCandidatePositions(obstacle_opt);
    if (!obstacle_filtered.empty()) {
        std::cerr << "wall candidates should be rejected by inflated obstacle filter\n";
        return 1;
    }

    const fs::path sparse_dir = fs::temp_directory_path() / "lightning_reloc_support_test";
    CloudPtr sparse_cloud(new PointCloudType);
    AddPoint(sparse_cloud, 0.0, 0.0, 0.0);
    AddPoint(sparse_cloud, 10.0, 0.0, 0.0);
    if (!WriteMap(sparse_dir, sparse_cloud)) {
        std::cerr << "failed to write sparse map\n";
        return 1;
    }

    auto sparse_map = LoadMap(sparse_dir);
    auto sparse_opt = TestFilterOptions(5.0);
    sparse_opt.clear_radius = 0.0;
    sparse_opt.support_radius = 0.2;
    sparse_opt.min_support_cells = 1;
    const auto sparse_filtered = sparse_map->GetRelocalizationCandidatePositions(sparse_opt);
    if (ContainsXY(sparse_filtered, 5.0, 0.0)) {
        std::cerr << "bbox-hole candidate should be rejected by support filter\n";
        return 1;
    }

    const fs::path corridor_dir = fs::temp_directory_path() / "lightning_reloc_corridor_test";
    CloudPtr corridor_cloud(new PointCloudType);
    for (double y : {-0.5, 0.0, 0.5}) {
        AddPoint(corridor_cloud, -0.5, y, 1.0);
        AddPoint(corridor_cloud, 0.5, y, 1.0);
    }
    if (!WriteMap(corridor_dir, corridor_cloud)) {
        std::cerr << "failed to write corridor map\n";
        return 1;
    }

    auto corridor_map = LoadMap(corridor_dir);
    auto corridor_opt = TestFilterOptions(0.5);
    corridor_opt.support_radius = 0.6;
    corridor_opt.min_support_cells = 2;
    const auto corridor_filtered = corridor_map->GetRelocalizationCandidatePositions(corridor_opt);
    if (!ContainsXY(corridor_filtered, 0.0, 0.0)) {
        std::cerr << "narrow corridor center should remain a valid candidate\n";
        return 1;
    }

    fs::remove_all(map_dir);
    fs::remove_all(obstacle_dir);
    fs::remove_all(sparse_dir);
    fs::remove_all(corridor_dir);
    return 0;
}
