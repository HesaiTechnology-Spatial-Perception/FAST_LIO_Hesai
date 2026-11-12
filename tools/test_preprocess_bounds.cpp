// Copyright 2026 Hesai Technology. SPDX-License-Identifier: GPL-2.0
// Run with FAST_LIO_PREPROCESS_ASAN_TEST=ON to detect writes beyond scan-line bounds.
#include "preprocess.h"
#include <iostream>
int main() {
    for (int count : {0, 1, 7, 8, 9, 10, 40, 128}) {
        for (float spacing : {0.001f, 0.1f}) {
            Preprocess pre;
            pre.set(true, JT32, 0.01, 1);
            pre.N_SCANS = 32;
            pre.time_unit = SEC;
            pcl::PointCloud<hesai_point_type::Point> cloud;
            for (int i = 0; i < count; ++i) {
                hesai_point_type::Point p{};
                p.x = 10.0f + spacing * i; p.y = 0; p.z = 0;
                p.intensity = 1; p.ring = 0; p.timestamp = 100.0 + 0.001 * i;
                cloud.push_back(p);
            }
#ifdef FAST_LIO_TEST_ROS2
            auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
#else
            sensor_msgs::PointCloud2::Ptr msg(new sensor_msgs::PointCloud2);
#endif
            pcl::toROSMsg(cloud, *msg);
            PointCloudXYZI::Ptr output(new PointCloudXYZI);
            pre.process(msg, output);
            for (const auto &point : *output)
                if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
                    return 1;
        }
    }
    std::cout << "Preprocess scan-line bounds regressions passed\n";
}
