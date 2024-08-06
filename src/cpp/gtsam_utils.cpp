#include "gtsam_utils.h"

namespace gtsamutils
{
    gtsam::Point2_ PredictLandmarkImageLocation(gtsam::Pose3_ worldTbody_fac,
                                                gtsam::Pose3 bodyPcamera,
                                                gtsam::Cal3_S2_ cameraCal,
                                                gtsam::Point3 worldPcorner)
    {
        using namespace gtsam;

        // world->camera pose as a composition of world->body factory and
        // body->camera factor
        const gtsam::Pose3_ worldTcamera_fac =
            Pose3_(worldTbody_fac, &Pose3::transformPoseFrom, Pose3_(bodyPcamera));
        // Camera->tag corner vector
        const gtsam::Point3_ camPcorner = transformTo(worldTcamera_fac, worldPcorner);
        // project from vector down to pinhole location, then uncalibrate to pixel
        // locations
        const gtsam::Point2_ prediction =
            uncalibrate<Cal3_S2>(cameraCal, project(camPcorner));

        return prediction;
    }

    gtsam::Point2_ PredictLandmarkImageLocationFactor(gtsam::Pose3_ worldTbody_fac,
                                                      gtsam::Pose3 bodyPcamera,
                                                      gtsam::Cal3_S2_ cameraCal,
                                                      gtsam::Point3_ worldPcorner)
    {
        // world->camera pose as a composition of world->body factory and
        // body->camera factor
        const gtsam::Pose3_ worldTcamera_fac =
            gtsam::Pose3_(worldTbody_fac, &gtsam::Pose3::transformPoseFrom, gtsam::Pose3_(bodyPcamera));
        // Camera->tag corner vector
        const gtsam::Point3_ camPcorner = transformTo(worldTcamera_fac, worldPcorner);
        // project from vector down to pinhole location, then uncalibrate to pixel
        // locations
        const gtsam::Point2_ prediction =
            gtsam::uncalibrate<gtsam::Cal3_S2>(cameraCal, project(camPcorner));

        return prediction;
    }
}