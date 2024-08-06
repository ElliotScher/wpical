#pragma once

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/expressions.h>
#include <nlohmann/json.hpp>

#include <map>
#include <optional>
#include <vector>

namespace tagmodel
{
    std::optional<std::vector<gtsam::Point3>> WorldToCorners(int id, nlohmann::json);
    std::vector<gtsam::Point3_> WorldToCornersFactor(gtsam::Pose3_ worldTtag);
    std::optional<gtsam::Pose3> worldToTag(int id, nlohmann::json json);
}