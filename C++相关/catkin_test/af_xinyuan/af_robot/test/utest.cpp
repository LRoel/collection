/*
 * utest.cpp
 *
 *  Created on: 2016年8月27日
 *      Author: shengsong
 */
#include <gtest/gtest.h>
#include "map_db.h"
#include "af_path/af_path_planner.h"

TEST(DatabaseTestSuite, testCase1)
{
    Map map;
    EXPECT_EQ(0, 0);
}

TEST(DatabaseTestSuite, testCase2)
{

}

TEST(PathPlannerTestSuite, testCase1)
{
    AFPathPlanner afpp;
    afpp.setPath(1,2);
    EXPECT_EQ(1, 1);
    EXPECT_EQ(2, 2);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

