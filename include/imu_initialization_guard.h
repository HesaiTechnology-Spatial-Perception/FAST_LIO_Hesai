// Copyright 2026 Hesai Technology. All rights reserved.
// SPDX-License-Identifier: GPL-2.0

#pragma once

#include <cmath>
#include <string>
#include <vector>

namespace fast_lio
{
struct ImuInitSample
{
    double stamp;
    double gyro_norm;  // rad/s, after the configured unit conversion
    double acc_norm;   // raw acceleration units; only finite/nonzero is checked
};

// Opt-in screening of a fixed initialization window, not a proof of stationarity.
// In particular, constant translation cannot be detected from these tests.
inline std::string check_imu_initialization(
    const std::vector<ImuInitSample> &samples, double start, double duration,
    double max_gyro_norm, bool complete)
{
    constexpr double max_gap = 0.05;  // also the tolerance at window boundaries
    if (!std::isfinite(start) || !std::isfinite(duration) || duration < 0.1 ||
        !std::isfinite(max_gyro_norm) || max_gyro_norm <= 0.0)
        return "guard requires a finite duration >= 0.1 s and gyro limit > 0 rad/s";
    double previous = start;
    for (std::size_t i = 0; i < samples.size(); ++i)
    {
        const auto &sample = samples[i];
        if (!std::isfinite(sample.stamp) || !std::isfinite(sample.gyro_norm) ||
            !std::isfinite(sample.acc_norm) || sample.acc_norm <= 0.0)
            return "non-finite IMU data or zero acceleration during initialization";
        if (sample.gyro_norm > max_gyro_norm)
            return "rotation exceeds initialization limit at sensor time " +
                std::to_string(sample.stamp) + ": " + std::to_string(sample.gyro_norm) +
                " rad/s > " + std::to_string(max_gyro_norm) + " rad/s";
        if (sample.stamp < start || sample.stamp > start + duration ||
            (i > 0 && sample.stamp <= previous) ||
            (i > 0 && sample.stamp - previous > max_gap))
            return "initialization IMU timestamps are incomplete, nonmonotonic or have a gap > 50 ms";
        previous = sample.stamp;
    }
    if (complete && (samples.size() < 11 || samples.empty() ||
                     samples.front().stamp - start > max_gap ||
                     start + duration - previous > max_gap))
        return "initialization window lacks IMU coverage (need >= 11 samples and both boundaries within 50 ms)";
    return {};
}
}  // namespace fast_lio
