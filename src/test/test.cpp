// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>
#include <cameracalibration.h>
#include <fieldcalibration.h>
#include <nlohmann/json.hpp>
#include <iostream>

const std::string projectRootPath = PROJECT_ROOT_PATH;

TEST(Camera_Calibration, OpenCV_Typical)
{
  int ret = cameracalibration::calibrate(projectRootPath + "/assets/testcalibration.mp4", 0.709, 0.551, 12, 8, false);
  EXPECT_EQ(ret, 0);
}

TEST(Camera_Calibration, OpenCV_Atypical)
{
  int ret = cameracalibration::calibrate(projectRootPath + "/assets/field video/long.mp4", 0.709, 0.551, 12, 8, false);
  EXPECT_EQ(ret, 1);
}

TEST(Camera_Calibration, MRcal_Typical)
{
  int ret = cameracalibration::calibrate(projectRootPath + "/assets/testcalibration.mp4", 0.709, 0.551, 12, 8, 1080, 1920, false);
  EXPECT_EQ(ret, 0);
}

TEST(Camera_Calibration, MRcal_Atypical)
{
  int ret = cameracalibration::calibrate(projectRootPath + "/assets/field video/short.mp4", 0.709, 0.551, 12, 8, 1080, 1920, false);
  EXPECT_EQ(ret, 1);
}

TEST(Field_Calibration, Typical)
{
  int ret = fieldcalibration::calibrate(projectRootPath + "/assets/field video", projectRootPath + "/assets", projectRootPath + "/assets/camera calibration.json", projectRootPath + "/assets/2024-crescendo.json", 3, 15, false);
  EXPECT_EQ(ret, 0);
}

TEST(Field_Calibration, Atypical_Bad_Camera_Model_Directory)
{
  int ret = fieldcalibration::calibrate(projectRootPath + "/assets/field video", projectRootPath + "/assets", projectRootPath + "/assets/field video/long.mp4", projectRootPath + "/assets/2024-crescendo.json", 3, 15, false);
  EXPECT_EQ(ret, 1);
}

TEST(Field_Calibration, Atypical_Bad_Ideal_JSON)
{
  int ret = fieldcalibration::calibrate(projectRootPath + "/assets/field video", projectRootPath + "/assets", projectRootPath + "/assets/camera calibration.json", projectRootPath + "/assets/camera calibration.json", 3, 15, false);
  EXPECT_EQ(ret, 1);
}

TEST(Field_Calibration, Atypical_Bad_Input_Directory)
{
  int ret = fieldcalibration::calibrate(projectRootPath + "/assets", projectRootPath + "/assets", projectRootPath + "/assets/camera calibration.json", projectRootPath + "/assets/2024-crescendo.json", 3, 15, false);
  EXPECT_EQ(ret, 1);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}