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

#include "cartographer/mapping_2d/submaps.h"

#include <map>
#include <string>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/common/port.h"
#include "cartographer/transform/transform.h"
#include "gmock/gmock.h"

namespace cartographer {
namespace mapping_2d {
namespace {

TEST(SubmapsTest, TheRightNumberOfScansAreInserted) {
  constexpr int kNumRangeData = 10;
  auto parameter_dictionary = common::MakeDictionary(
      "return {"
      "resolution = 0.05, "
      "half_length = 10., "
      "num_range_data = " +
      std::to_string(kNumRangeData) +
      ", "
      "output_debug_images = false, "
      "range_data_inserter = {"
      "insert_free_space = true, "
      "hit_probability = 0.53, "
      "miss_probability = 0.495, "
      "},"
      "}");
  Submaps submaps{CreateSubmapsOptions(parameter_dictionary.get())};
  auto num_inserted = [&submaps](const int i) {
    return submaps.Get(i)->end_range_data_index -
           submaps.Get(i)->begin_range_data_index;
  };
  for (int i = 0; i != 1000; ++i) {
    submaps.InsertRangeData({Eigen::Vector3f::Zero(), {}, {}});
    const int matching = submaps.matching_index();
    // Except for the first, maps should only be returned after enough scans.
    /**
    * @brief 除了第一个，地图应该只有在足够的扫描后才能返回.
    */
    if (matching != 0) {
      EXPECT_LE(kNumRangeData, num_inserted(matching));
    }
  }
  for (int i = 0; i != submaps.size() - 2; ++i) {
    // Submaps should not be left without the right number of scans in them.
    /**
    * @brief 如果没有正确的扫描次数，则不应该留下子图.
    */
    EXPECT_EQ(kNumRangeData * 2, num_inserted(i));
    EXPECT_EQ(i * kNumRangeData, submaps.Get(i)->begin_range_data_index);
    EXPECT_EQ((i + 2) * kNumRangeData, submaps.Get(i)->end_range_data_index);
  }
}

}  // namespace
}  // namespace mapping_2d
}  // namespace cartographer
