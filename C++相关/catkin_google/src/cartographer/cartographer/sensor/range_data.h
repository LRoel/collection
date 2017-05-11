/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_SENSOR_RANGE_DATA_H_
#define CARTOGRAPHER_SENSOR_RANGE_DATA_H_

#include "cartographer/common/port.h"
#include "cartographer/sensor/compressed_point_cloud.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/proto/sensor.pb.h"

namespace cartographer {
    namespace sensor {

// Rays begin at 'origin'. 'returns' are the points where laser returns were
// detected. 'misses' are points in the direction of rays for which no return
// was detected, and were inserted at a configured distance. It is assumed that
// between the 'origin' and 'misses' is free space.
/**
* @brief 光线从'起源'开始.“返回”是检测到激光返回的点.“未命中”是在没有检测到返回的射线方向上的点，并以配置的距离插入.假设“起源”和“未命中”是自由空间.
*/
        struct RangeData {
            Eigen::Vector3f origin;
            PointCloud returns;
            PointCloud misses;

            // Reflectivity value of returns.
            /**
			* @brief 回报的反射率值.
			*/
            std::vector<uint8> reflectivities;
        };

        struct GPSData {
            Eigen::Vector3f gpspose;
	        Eigen::Vector3f gpsvar;
        };

// Builds a PointCloud of returns from 'proto', dropping any beams with ranges
// outside the valid range described by 'proto'.
/**
* @brief 从“proto”建立点云的返回，将任何波束放在“proto”所描述​​的有效范围之外.
*/
        PointCloud ToPointCloud(const proto::LaserScan& proto);

// Like above, but also extracts intensities of ouf the laser scan. The
// intensities of the laser are device specific and therefore require
// normalization to be comparable. In case the 'proto' does not contain
// intensities, this will return all 0. for the intensities.
/**
* @brief 如上所述，也提取了激光扫描的强度.激光的强度是器件特异性的，因此要求归一化是可比的.在“proto”不包含强度的情况下，这将返回0.为强度.
*/
        PointCloudWithIntensities ToPointCloudWithIntensities(
                const proto::LaserScan& proto);

// Converts 'range_data' to a proto::RangeData.
/**
* @brief 将“range_data”转换为proto :: RangeData.
*/
        proto::RangeData ToProto(const RangeData& range_data);

// Converts 'proto' to a RangeData.
/**
* @brief 将'proto'转换为RangeData.
*/
        RangeData FromProto(const proto::RangeData& proto);

        RangeData TransformRangeData(const RangeData& range_data,
                                     const transform::Rigid3f& transform);

// Crops 'range_data' according to the region defined by 'min_z' and 'max_z'.
/**
* @brief 根据由'min_z'和'max_z'定义的区域的作物的range_data'.
*/
        RangeData CropRangeData(const RangeData& range_data, float min_z, float max_z);

// Like RangeData but with compressed point clouds. The point order changes
// when converting from RangeData.
/**
* @brief 像RangeData，但使用压缩点云.从RangeData转换时，点顺序发生变化.
*/
        struct CompressedRangeData {
            Eigen::Vector3f origin;
            CompressedPointCloud returns;
            CompressedPointCloud misses;

            // Reflectivity value of returns.
            /**
			* @brief 回报的反射率值.
			*/
            std::vector<uint8> reflectivities;
        };

        CompressedRangeData Compress(const RangeData& range_data);

        RangeData Decompress(const CompressedRangeData& compressed_range_Data);

    }  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_RANGE_DATA_H_
