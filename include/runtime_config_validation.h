// Copyright 2026 Hesai Technology. All rights reserved.
// SPDX-License-Identifier: GPL-2.0

#pragma once

#include <cmath>
#include <string>
#include <vector>

namespace fast_lio
{
inline std::string check_preprocess_config(
    int scan_line, int point_filter_num, int timestamp_unit, double blind)
{
    if (scan_line < 1 || scan_line > 128)
        return "preprocess.scan_line must be in [1, 128]";
    if (point_filter_num < 1)
        return "point_filter_num must be a positive integer";
    if (timestamp_unit < 0 || timestamp_unit > 3)
        return "preprocess.timestamp_unit must be 0 (s), 1 (ms), 2 (us), or 3 (ns)";
    if (!std::isfinite(blind) || blind < 0.0)
        return "preprocess.blind must be finite and non-negative";
    return {};
}

inline std::string check_extrinsics(
    const std::vector<double> &translation, const std::vector<double> &rotation)
{
    if (translation.size() != 3)
        return "mapping.extrinsic_T must contain exactly 3 values";
    if (rotation.size() != 9)
        return "mapping.extrinsic_R must contain exactly 9 values";
    for (double value : translation)
        if (!std::isfinite(value))
            return "mapping.extrinsic_T contains a non-finite value";
    for (double value : rotation)
        if (!std::isfinite(value))
            return "mapping.extrinsic_R contains a non-finite value";

    constexpr double tolerance = 1e-3;
    for (int row = 0; row < 3; ++row)
    {
        double norm = 0.0;
        for (int col = 0; col < 3; ++col)
            norm += rotation[row * 3 + col] * rotation[row * 3 + col];
        if (std::fabs(norm - 1.0) > tolerance)
            return "mapping.extrinsic_R rows must be unit length";
    }
    for (int first = 0; first < 3; ++first)
    {
        for (int second = first + 1; second < 3; ++second)
        {
            double dot = 0.0;
            for (int col = 0; col < 3; ++col)
                dot += rotation[first * 3 + col] * rotation[second * 3 + col];
            if (std::fabs(dot) > tolerance)
                return "mapping.extrinsic_R rows must be orthogonal";
        }
    }
    const double det =
        rotation[0] * (rotation[4] * rotation[8] - rotation[5] * rotation[7]) -
        rotation[1] * (rotation[3] * rotation[8] - rotation[5] * rotation[6]) +
        rotation[2] * (rotation[3] * rotation[7] - rotation[4] * rotation[6]);
    if (std::fabs(det - 1.0) > tolerance)
        return "mapping.extrinsic_R must have determinant +1";
    return {};
}
}  // namespace fast_lio
