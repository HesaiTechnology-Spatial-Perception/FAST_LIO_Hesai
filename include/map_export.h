// Copyright 2026 Hesai Technology. SPDX-License-Identifier: GPL-2.0
#pragma once

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <cstdio>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>
#include <sys/stat.h>
#include <unistd.h>

namespace fastlio {

inline std::string json_quote(const std::string &s) {
    std::ostringstream out;
    out << '"';
    for (unsigned char c : s) {
        if (c == '"' || c == '\\') out << '\\' << c;
        else if (c < 32) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c);
        else out << c;
    }
    return out.str() + '"';
}

inline void make_parent_directories(const std::string &path) {
    for (size_t end = path.find('/', 1); end != std::string::npos; end = path.find('/', end + 1)) {
        const auto directory = path.substr(0, end);
        if (::mkdir(directory.c_str(), 0755) != 0 && errno != EEXIST)
            throw std::runtime_error("Cannot create " + directory + ": " + std::strerror(errno));
        struct stat st{};
        if (::stat(directory.c_str(), &st) != 0 || !S_ISDIR(st.st_mode))
            throw std::runtime_error("Not a directory: " + directory);
    }
}

inline std::string temporary_file(const std::string &prefix) {
    std::vector<char> name(prefix.begin(), prefix.end());
    const char suffix[] = ".tmp-XXXXXX";
    name.insert(name.end(), suffix, suffix + sizeof(suffix));
    const int fd = ::mkstemp(name.data());
    if (fd < 0) throw std::runtime_error("Cannot create temporary file beside " + prefix + ": " + std::strerror(errno));
    ::close(fd);
    return name.data();
}

// Publish a complete file with rename; a failed write never truncates the last snapshot.
template <typename Writer>
void atomic_write(const std::string &path, Writer write) {
    make_parent_directories(path);
    const std::string temp = temporary_file(path);
    try {
        std::ofstream out(temp, std::ios::binary | std::ios::trunc);
        if (!out) throw std::runtime_error("Cannot open " + temp);
        write(out);
        out.flush();
        if (!out) throw std::runtime_error("Write failed (check free disk space): " + path);
        out.close();
        if (!out) throw std::runtime_error("Close failed: " + path);
        if (::rename(temp.c_str(), path.c_str()) != 0)
            throw std::runtime_error("Cannot publish " + path + ": " + std::strerror(errno));
    } catch (...) {
        std::remove(temp.c_str());
        throw;
    }
}

struct MapPoint { float x, y, z, intensity; };
static_assert(sizeof(MapPoint) == 16, "PCD XYZI record must contain four float32 values");

class TimestampSampler {
public:
    bool accept(double stamp, double hz = 1.0) {
        if (!std::isfinite(stamp) || !std::isfinite(hz) || hz <= 0)
            throw std::invalid_argument("Sampling requires a finite timestamp and positive frequency");
        if (!started_) { started_ = true; first_ = stamp; bin_ = 0; return true; }
        const double bin = std::floor((stamp - first_) * hz + 1e-7);
        if (bin <= bin_) return false;
        bin_ = bin;
        return true;
    }
private:
    bool started_ = false;
    double first_ = 0, bin_ = 0;
};

class VoxelAccumulator {
    struct Key {
        int64_t x, y, z;
        bool operator==(const Key &k) const { return x == k.x && y == k.y && z == k.z; }
    };
    struct Hash {
        size_t operator()(const Key &k) const {
            size_t h = std::hash<int64_t>{}(k.x);
            h ^= std::hash<int64_t>{}(k.y) + size_t(0x9e3779b9) + (h << 6) + (h >> 2);
            return h ^ (std::hash<int64_t>{}(k.z) + size_t(0x9e3779b9) + (h << 6) + (h >> 2));
        }
    };
    struct Cell { double x = 0, y = 0, z = 0, intensity = 0; uint64_t count = 0; };
public:
    explicit VoxelAccumulator(double leaf = 0.1) { set_leaf(leaf); }
    void set_leaf(double leaf) {
        if (!std::isfinite(leaf) || leaf <= 0) throw std::invalid_argument("Map voxel size must be finite and positive");
        if (!cells_.empty()) throw std::logic_error("Cannot change voxel size after accumulation");
        leaf_ = leaf;
    }
    void add(const MapPoint &p) {
        const double x = std::floor(p.x / leaf_), y = std::floor(p.y / leaf_), z = std::floor(p.z / leaf_);
        constexpr double limit = 9e18;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
            std::abs(x) > limit || std::abs(y) > limit || std::abs(z) > limit)
            throw std::runtime_error("Invalid map point or voxel index; mapping may have diverged");
        auto &c = cells_[Key{int64_t(x), int64_t(y), int64_t(z)}];
        // Preserve the weight of every original observation across frames and saves.
        const double n = static_cast<double>(++c.count);
        c.x += (p.x - c.x) / n; c.y += (p.y - c.y) / n; c.z += (p.z - c.z) / n;
        c.intensity += (p.intensity - c.intensity) / n;
    }
    size_t size() const { return cells_.size(); }
    template <typename Consumer> void for_each(Consumer consume) const {
        for (const auto &entry : cells_) {
            const auto &c = entry.second;
            consume(MapPoint{float(c.x), float(c.y), float(c.z), float(c.intensity)});
        }
    }
private:
    double leaf_ = 0.1;
    std::unordered_map<Key, Cell, Hash> cells_;
};

// Raw and 1 Hz variants spool compact XYZI records to disk instead of retaining
// every scan in RAM. The spool is private; only complete PCD snapshots are published.
class PointSpool {
public:
    ~PointSpool() { stream_.close(); if (!path_.empty()) std::remove(path_.c_str()); }
    void open(const std::string &prefix) {
        make_parent_directories(prefix);
        path_ = temporary_file(prefix + ".buffer");
        stream_.open(path_, std::ios::binary | std::ios::trunc);
        if (!stream_) throw std::runtime_error("Cannot open map spool: " + path_);
    }
    void append(const std::vector<MapPoint> &points) {
        stream_.write(reinterpret_cast<const char *>(points.data()), points.size() * sizeof(MapPoint));
        if (!stream_) throw std::runtime_error("Map spool write failed; check disk space: " + path_);
        count_ += points.size();
    }
    void copy_to(std::ostream &out) {
        stream_.flush();
        if (!stream_) throw std::runtime_error("Map spool flush failed: " + path_);
        std::ifstream in(path_, std::ios::binary);
        if (!in) throw std::runtime_error("Cannot read map spool: " + path_);
        char buffer[65536];
        uint64_t bytes = 0;
        while (in.read(buffer, sizeof(buffer)) || in.gcount()) {
            bytes += uint64_t(in.gcount()); out.write(buffer, in.gcount());
            if (!out) throw std::runtime_error("PCD snapshot write failed; check disk space");
        }
        if (in.bad() || bytes != count_ * sizeof(MapPoint)) throw std::runtime_error("Map spool size mismatch");
    }
    uint64_t size() const { return count_; }
private:
    std::string path_;
    std::ofstream stream_;
    uint64_t count_ = 0;
};

inline void pcd_header(std::ostream &out, uint64_t points) {
    out << "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n"
           "TYPE F F F F\nCOUNT 1 1 1 1\nWIDTH " << points <<
           "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << points << "\nDATA binary\n";
}

class MapExporter {
public:
    void configure(const std::string &path, const std::string &mode, double leaf) {
        if (mode != "raw" && mode != "voxel" && mode != "1hz" && mode != "all")
            throw std::invalid_argument("pcd_save.mode must be raw, voxel, 1hz, or all");
        if (path.empty()) throw std::invalid_argument("Map output path is empty");
        path_ = path; mode_ = mode; leaf_ = leaf;
        if (mode == "voxel" || mode == "all") voxels_.set_leaf(leaf);
        make_parent_directories(path);
        if (mode == "raw" || mode == "all") raw_.open(path + ".raw");
        if (mode == "1hz" || mode == "all") sampled_.open(path + ".1hz");
    }
    template <typename Cloud> bool add_frame(const Cloud &cloud, double stamp) {
        if (!std::isfinite(stamp)) throw std::runtime_error("Non-finite map timestamp");
        if (frames_ && stamp < last_stamp_) throw std::runtime_error("Map timestamps moved backwards; restart mapping");
        if (frames_ && stamp == last_stamp_) return false;
        std::vector<MapPoint> points;
        points.reserve(cloud.size());
        for (const auto &p : cloud) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || !std::isfinite(p.intensity)) {
                ++invalid_points_; continue;
            }
            points.push_back(MapPoint{p.x, p.y, p.z, p.intensity});
        }
        if (points.empty()) return false;
        if (mode_ == "raw" || mode_ == "all") raw_.append(points);
        if (mode_ == "voxel" || mode_ == "all") for (const auto &p : points) voxels_.add(p);
        if ((mode_ == "1hz" || mode_ == "all") && sampler_.accept(stamp)) {
            sampled_.append(points); ++sampled_frames_;
        }
        if (frames_++ == 0) first_stamp_ = stamp;
        last_stamp_ = stamp; input_points_ += points.size(); dirty_ = true;
        return true;
    }
    bool dirty() const { return dirty_; }
    uint64_t frames() const { return frames_; }
    uint64_t sampled_frames() const { return sampled_frames_; }
    uint64_t input_points() const { return input_points_; }
    size_t voxel_count() const { return voxels_.size(); }
    std::string manifest_path() const { return path_ + ".json"; }
    std::string report_json() const {
        std::ostringstream out;
        out << std::setprecision(17) << "{\"mode\":" << json_quote(mode_)
            << ",\"leaf_size\":" << leaf_ << ",\"frames\":" << frames_
            << ",\"sampled_frames\":" << sampled_frames_ << ",\"input_points\":" << input_points_
            << ",\"invalid_points\":" << invalid_points_ << ",\"first_stamp\":" << first_stamp_
            << ",\"last_stamp\":" << last_stamp_ << ",\"files\":[";
        bool first = true;
        for_outputs([&](const std::string &path, uint64_t count, const std::string &kind) {
            if (!first) out << ',';
            first = false;
            out << "{\"path\":" << json_quote(path) << ",\"points\":" << count << ",\"kind\":" << json_quote(kind) << '}';
        });
        return out.str() + "]}";
    }
    bool save(std::string &message) {
        if (!frames_) { message = "No registered frames buffered"; return false; }
        try {
            if (dirty_) {
                for_outputs([&](const std::string &path, uint64_t count, const std::string &kind) {
                    atomic_write(path, [&](std::ostream &out) {
                        pcd_header(out, count);
                        if (kind == "voxel") voxels_.for_each([&](const MapPoint &p) { out.write(reinterpret_cast<const char *>(&p), sizeof(p)); });
                        else if (kind == "raw") raw_.copy_to(out);
                        else sampled_.copy_to(out);
                    });
                });
                atomic_write(manifest_path(), [&](std::ostream &out) { out << report_json() << '\n'; });
                dirty_ = false;
            }
            message = report_json();
            return true;
        } catch (const std::exception &e) { message = e.what(); return false; }
    }
private:
    template <typename Consumer> void for_outputs(Consumer consume) const {
        if (mode_ != "all") {
            consume(path_, mode_ == "voxel" ? voxels_.size() : mode_ == "raw" ? raw_.size() : sampled_.size(), mode_);
        } else {
            const auto stem = path_.size() >= 4 && path_.substr(path_.size() - 4) == ".pcd" ? path_.substr(0, path_.size() - 4) : path_;
            consume(path_, voxels_.size(), "voxel");
            consume(stem + "_origin.pcd", raw_.size(), "raw");
            consume(stem + "_1hz.pcd", sampled_.size(), "1hz");
        }
    }
    std::string path_, mode_;
    double leaf_ = 0.1, first_stamp_ = 0, last_stamp_ = 0;
    uint64_t frames_ = 0, sampled_frames_ = 0, input_points_ = 0, invalid_points_ = 0;
    bool dirty_ = false;
    VoxelAccumulator voxels_;
    TimestampSampler sampler_;
    PointSpool raw_, sampled_;
};
} // namespace fastlio
