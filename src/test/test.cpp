// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>
#include <cameracalibration.h>
#include <nlohmann/json.hpp>

TEST(OpenCV_Calibration, Typical)
{
  const std::string projectRootPath = PROJECT_ROOT_PATH;

  nlohmann::json ret = cameracalibration::calibrate(projectRootPath + "/assets/testcalibration.mp4", 0.709, 0.551, 12, 8, false);
  EXPECT_EQ(ret, 0);
}

TEST(OpenCV_Calibration, Atypical)
{
  const std::string projectRootPath = PROJECT_ROOT_PATH;

  nlohmann::json ret = cameracalibration::calibrate(projectRootPath + "assets/field video/PXL_20240730_204208090.mp4", 0.709, 0.551, 12, 8, true);
  EXPECT_EQ(ret, 1);
}

// TEST(CameraCalibration, MRcal)
// {
//   const std::string projectRootPath = PROJECT_ROOT_PATH;

//   nlohmann::json ret = cameracalibration::calibrate(projectRootPath + "/assets/testcalibration.mp4", 0.709, 12, 8, 1080, 1920, false);
//   EXPECT_TRUE(ret.is_object());
// }

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}