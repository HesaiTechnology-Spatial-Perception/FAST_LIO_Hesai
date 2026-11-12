// Copyright 2026 Hesai Technology. All rights reserved.
// SPDX-License-Identifier: GPL-2.0

#include "imu_initialization_guard.h"
#include <cassert>
#include <iostream>
#include <limits>

int main(int argc, char **argv)
{
    // Optional offline bag test: START DURATION GYRO_LIMIT, then stdin rows
    // "sensor_stamp gyro_norm_rad_s acceleration_norm".
    if (argc == 4)
    {
        std::vector<fast_lio::ImuInitSample> input;
        fast_lio::ImuInitSample sample;
        while (std::cin >> sample.stamp >> sample.gyro_norm >> sample.acc_norm)
            input.push_back(sample);
        const auto error = fast_lio::check_imu_initialization(
            input, std::stod(argv[1]), std::stod(argv[2]), std::stod(argv[3]), true);
        std::cout << (error.empty() ? "ACCEPT" : "REJECT: " + error)
                  << ", samples=" << input.size() << std::endl;
        return error.empty() ? 0 : 2;
    }
    std::vector<fast_lio::ImuInitSample> samples;
    for (int i = 0; i < 100; ++i)
        samples.push_back({10.0 + i * 0.005, 0.003, 9.81});
    const auto check = [](const std::vector<fast_lio::ImuInitSample> &s, bool complete = true) {
        return fast_lio::check_imu_initialization(s, 10.0, 0.5, 0.03, complete);
    };
    assert(check(samples).empty());
    auto bad = samples;
    bad[10].gyro_norm = 0.0867031;  // underground 00's suspicious initialization
    assert(!check(bad, false).empty());
    bad = samples;
    bad.erase(bad.begin(), bad.begin() + 20);
    assert(!check(bad).empty());  // lost prefix
    bad = samples;
    bad.erase(bad.begin() + 20, bad.begin() + 40);
    assert(!check(bad).empty());  // interior gap
    bad = samples;
    bad.resize(11);
    assert(check(bad, false).empty());
    assert(!check(bad).empty());  // count alone is insufficient
    bad = samples;
    bad[10].stamp = bad[9].stamp;
    assert(!check(bad).empty());
    bad = samples;
    bad[10].gyro_norm = std::numeric_limits<double>::quiet_NaN();
    assert(!check(bad).empty());
    bad = samples;
    bad[10].acc_norm = 0;
    assert(!check(bad).empty());
    assert(!check({}).empty());
    assert(!fast_lio::check_imu_initialization(samples, 10.0, 0.0, 0.03, true).empty());
    std::cout << "IMU initialization guard tests passed\n";
}
