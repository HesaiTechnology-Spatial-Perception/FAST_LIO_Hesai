// Copyright 2026 Hesai Technology. SPDX-License-Identifier: GPL-2.0
#pragma once
#include <cstdint>
#include <stdexcept>
#include <string>

namespace fastlio {
template <typename CloudMessage> void validate_hesai_cloud(const CloudMessage &msg) {
    struct Required { const char *name; int type; uint32_t size; };
    const Required fields[] = {{"x", 7, 4}, {"y", 7, 4}, {"z", 7, 4},
                               {"intensity", 7, 4}, {"ring", 4, 2}, {"timestamp", 8, 8}};
    for (const auto &expected : fields) {
        bool valid = false;
        for (const auto &actual : msg.fields) {
            if (actual.name == expected.name) {
                valid = actual.datatype == expected.type && actual.count == 1 &&
                        uint64_t(actual.offset) + expected.size <= msg.point_step;
                break;
            }
        }
        if (!valid) throw std::invalid_argument(std::string("Invalid/missing PointCloud2 field: ") + expected.name);
    }
    if (uint64_t(msg.row_step) < uint64_t(msg.width) * msg.point_step ||
        uint64_t(msg.row_step) * msg.height > msg.data.size())
        throw std::invalid_argument("PointCloud2 payload/stride mismatch");
}
} // namespace fastlio
