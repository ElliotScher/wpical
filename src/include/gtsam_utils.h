#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/expressions.h>

namespace gtsamutils
{
    gtsam::Point2_ PredictLandmarkImageLocation(gtsam::Pose3_ worldTbody_fac,
                                                gtsam::Pose3 bodyPcamera,
                                                gtsam::Cal3_S2_ cameraCal,
                                                gtsam::Point3 worldPcorner);

    gtsam::Point2_ PredictLandmarkImageLocationFactor(gtsam::Pose3_ worldTbody_fac,
                                                      gtsam::Pose3 bodyPcamera,
                                                      gtsam::Cal3_S2_ cameraCal,
                                                      gtsam::Point3_ worldPcorner);
}