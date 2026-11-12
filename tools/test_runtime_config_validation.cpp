// Copyright 2026 Hesai Technology. All rights reserved.
// SPDX-License-Identifier: GPL-2.0

#include "runtime_config_validation.h"

#include <cassert>
#include <limits>
#include <vector>

int main()
{
    const std::vector<double> translation{0.0, 0.0, 0.0};
    const std::vector<double> identity{
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0};

    assert(fast_lio::check_preprocess_config(128, 2, 0, 0.5).empty());
    assert(!fast_lio::check_preprocess_config(0, 2, 0, 0.5).empty());
    assert(!fast_lio::check_preprocess_config(129, 2, 0, 0.5).empty());
    assert(!fast_lio::check_preprocess_config(128, 0, 0, 0.5).empty());
    assert(!fast_lio::check_preprocess_config(128, 2, 4, 0.5).empty());
    assert(!fast_lio::check_preprocess_config(
        128, 2, 0, std::numeric_limits<double>::quiet_NaN()).empty());

    assert(fast_lio::check_extrinsics(translation, identity).empty());
    assert(!fast_lio::check_extrinsics({}, identity).empty());
    assert(!fast_lio::check_extrinsics(translation, {}).empty());
    auto bad = identity;
    bad[1] = 0.5;  // determinant remains 1, but the rows are not orthonormal.
    assert(!fast_lio::check_extrinsics(translation, bad).empty());
    bad = identity;
    bad[0] = -1.0;  // reflection, determinant -1.
    assert(!fast_lio::check_extrinsics(translation, bad).empty());
    bad = identity;
    bad[0] = std::numeric_limits<double>::infinity();
    assert(!fast_lio::check_extrinsics(translation, bad).empty());
    return 0;
}
