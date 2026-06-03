# Third-Party Notices

This project (FAST_LIO_Hesai) is a fork of and derivative work based on
third-party open-source software. This file documents those components,
their authors, and their license terms, as required by the respective licenses.

---

## 1. FAST_LIO / FAST-LIO2

**Source**: https://github.com/hku-mars/FAST_LIO  
**Authors**: Wei Xu, Yixi Cai, Dongjiao He, Jiarong Lin, Fu Zhang — MARS Lab, HKU  
**License**: GNU General Public License v2 (GPL-2.0)  
**Files derived from this work**: `src/laserMapping.cpp`, `src/preprocess.cpp`, `src/preprocess.h`, `src/IMU_Processing.hpp`, `include/common_lib.h`, `include/so3_math.h`, `include/use-ikfom.hpp`, `include/Exp_mat.h`, `CMakeLists.txt`, `package.xml`

**Reference paper**:
> W. Xu, Y. Cai, D. He, J. Lin, and F. Zhang, "FAST-LIO2: Fast Direct
> LiDAR-Inertial Odometry," IEEE Transactions on Robotics, 2022.

The full GPL-2.0 license text is provided in the `LICENSE` file at the root
of this repository. Original copyright notices within the source files are
retained as required by GPL-2.0 §1.

---

## 2. LOAM (LiDAR Odometry and Mapping)

**Authors**: Ji Zhang, Carnegie Mellon University; further contributions
copyright (c) 2016, Southwest Research Institute  
**License**: BSD 3-Clause (see copyright notice in `src/laserMapping.cpp`)

**Reference paper**:
> J. Zhang and S. Singh, "LOAM: Lidar Odometry and Mapping in Real-time,"
> Robotics: Science and Systems Conference (RSS), Berkeley, CA, July 2014.

---

## 3. Livox Contributions

**Modifier**: Livox (dev@livoxtech.com)  
**License**: BSD (see copyright notice in `src/laserMapping.cpp`)

The original Livox modifications to laserMapping.cpp are retained as
required by the BSD license.

---

## 4. ikd-Tree

**Source**: https://github.com/hku-mars/ikd-Tree  
**Authors**: Yixi Cai, Wei Xu, Fu Zhang — MARS Lab, HKU  
**License**: See `include/ikd-Tree/` (MIT / BSD; refer to the submodule for
the current license text)  
**Included as**: git submodule at `include/ikd-Tree/`

**Reference paper**:
> Y. Cai, W. Xu, and F. Zhang, "ikd-Tree: An Incremental KD Tree for
> Robotic Applications," arXiv:2102.10808, 2021.

---

## 5. IKFoM (Iterated Kalman Filters on Manifolds)

**Source**: https://github.com/hku-mars/IKFoM  
**Authors**: Dongjiao HE — The University of Hong Kong  
**License**: BSD 3-Clause  
**Included as**: header-only toolkit at `include/IKFoM_toolkit/`

Copyright (c) 2019–2023, The University of Hong Kong. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
3. Neither the name of the University of Hong Kong nor the names of its
   contributors may be used to endorse or promote products derived from this
   software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED.

---

## Hesai Modifications

Files newly created by Hesai Technology (not derived from upstream):

| File | Description |
| ---- | ----------- |
| `config/jt16.yaml` | FAST-LIO2 configuration for Hesai JT16 (ROS 2) |
| `config/jt128.yaml` | FAST-LIO2 configuration for Hesai JT128 (ROS 2) |
| `launch/mapping_jt16.launch.py` | ROS 2 launch file for JT16 |
| `launch/mapping_jt128.launch.py` | ROS 2 launch file for JT128 |
| `tools/check_input.py` | Runtime input validation script |
| `tools/check_config.py` | Static yaml config validation script |
| `tools/check_map.py` | Map quality analysis script |
| `tools/pcap_to_rosbag/pcap_to_rosbag_ros1.sh` | PCAP → rosbag converter (ROS 1) |
| `tools/pcap_to_rosbag/pcap_to_rosbag_ros2.sh` | PCAP → rosbag converter (ROS 2) |
| `tools/README.md` | Tools documentation |
| `doc/results/fast_lio2_jt_demo*.jpg` | Demo result images |

Files substantially modified by Hesai Technology from upstream:

| File | Upstream origin | Hesai changes |
| ---- | --------------- | ------------- |
| `src/preprocess.h` | FAST_LIO upstream | Rewritten for JT16/JT128; removed non-Hesai handlers |
| `src/preprocess.cpp` | FAST_LIO upstream | Rewritten for JT16/JT128; removed non-Hesai handlers |
| `src/laserMapping.cpp` | FAST_LIO upstream | ROS 2 port; map_save service; imu_gyr_unit parameter |

All Hesai modifications are released under GPL-2.0, consistent with the
upstream FAST_LIO license.
