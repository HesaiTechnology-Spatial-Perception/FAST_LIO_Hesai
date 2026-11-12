// Copyright 2026 Hesai Technology. SPDX-License-Identifier: GPL-2.0
#include "map_export.h"
#include <cassert>
#include <iostream>

std::vector<fastlio::MapPoint> read_pcd(const std::string &path) {
    std::ifstream f(path, std::ios::binary);
    std::string line; size_t count = 0;
    while (std::getline(f, line) && line != "DATA binary")
        if (line.compare(0, 7, "POINTS ") == 0) count = std::stoull(line.substr(7));
    assert(f && count);
    std::vector<fastlio::MapPoint> points(count);
    f.read(reinterpret_cast<char *>(points.data()), count * sizeof(points[0]));
    assert(size_t(f.gcount()) == count * sizeof(points[0]));
    assert(f.peek() == std::char_traits<char>::eof());
    return points;
}

int main(int argc, char **argv) {
    assert(argc == 2);
    fastlio::make_parent_directories(std::string(argv[1]) + "/placeholder");
    std::string pattern = std::string(argv[1]) + "/export-XXXXXX";
    std::vector<char> directory(pattern.begin(), pattern.end()); directory.push_back('\0');
    assert(::mkdtemp(directory.data()));
    const std::string folder = std::string(directory.data()) + "/";
    const std::string path = folder + "map.pcd";
    fastlio::MapExporter exporter;
    exporter.configure(path, "all", 1.0);
    std::vector<fastlio::MapPoint> many(100, {0.1f, 0.2f, 0.3f, 1.0f});
    assert(exporter.add_frame(many, 100));
    assert(!exporter.add_frame(many, 100)); // duplicate scan cannot grow the map
    std::string report;
    assert(exporter.save(report));
    assert(!exporter.dirty());
    std::vector<fastlio::MapPoint> one(1, {0.9f, 0.2f, 0.3f, 2.0f});
    exporter.add_frame(one, 100.1);
    exporter.add_frame(one, 101.01);
    assert(exporter.save(report));
    assert(exporter.save(report)); // repeated saves are idempotent
    auto voxel = read_pcd(path);
    assert(voxel.size() == 1);
    assert(std::abs(voxel[0].x - (100 * 0.1 + 2 * 0.9) / 102) < 1e-6);
    assert(read_pcd(folder + "map_origin.pcd").size() == 102);
    assert(read_pcd(folder + "map_1hz.pcd").size() == 101);
    assert(exporter.frames() == 3 && exporter.sampled_frames() == 2);
    bool rejected = false;
    try { exporter.add_frame(one, 99); } catch (const std::runtime_error &) { rejected = true; }
    assert(rejected);
    // A writer failure must leave an existing destination untouched.
    const auto before = read_pcd(path);
    try {
        fastlio::atomic_write(path, [](std::ostream &out) {
            out << "partial";
            throw std::runtime_error("injected write failure");
        });
        assert(false);
    } catch (const std::runtime_error &) {}
    assert(read_pcd(path)[0].x == before[0].x);
    // Save failure retains the dirty data so a corrected path can be retried.
    const std::string blocked = folder + "blocked.pcd";
    ::mkdir(blocked.c_str(), 0755);
    fastlio::MapExporter retry;
    retry.configure(blocked, "voxel", 0.1);
    retry.add_frame(one, 1);
    assert(!retry.save(report) && retry.dirty());
    assert(::rmdir(blocked.c_str()) == 0);
    assert(retry.save(report) && !retry.dirty());
    fastlio::TimestampSampler sampler;
    int accepted = 0;
    for (int i = 0; i < 100; ++i) accepted += sampler.accept(1000.0 + i * 0.1);
    assert(accepted == 10); // determined by sensor stamps, independent of replay speed
    std::cout << "Map export tests passed (weighted voxel, raw, 1 Hz, duplicate, retry, atomic failure).\n";
}
