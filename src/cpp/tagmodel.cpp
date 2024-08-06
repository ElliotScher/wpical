#include <tagmodel.h>
#include <nlohmann/json.hpp>

using gtsam::Cal3_S2;
using gtsam::Point3;
using gtsam::Point3_;
using gtsam::Pose3;
using gtsam::Pose3_;
using gtsam::Rot3;
using std::map;
using std::vector;

namespace
{
    map<int, gtsam::Pose3> TagLayoutToMap(nlohmann::json json)
    {
        map<int, gtsam::Pose3> worldTtags;

        for (auto &tag : json["tags"].items())
        {
            double tagXPos = tag.value()["pose"]["translation"]["x"];
            double tagYPos = tag.value()["pose"]["translation"]["y"];
            double tagZPos = tag.value()["pose"]["translation"]["z"];
            double tagWQuat = tag.value()["pose"]["rotation"]["quaternion"]["W"];
            double tagXQuat = tag.value()["pose"]["rotation"]["quaternion"]["X"];
            double tagYQuat = tag.value()["pose"]["rotation"]["quaternion"]["Y"];
            double tagZQuat = tag.value()["pose"]["rotation"]["quaternion"]["Z"];

            gtsam::Rot3 rotation(tagWQuat, tagXQuat, tagYQuat, tagZQuat);
            gtsam::Point3 translation(tagXPos, tagYPos, tagZPos);

            worldTtags[tag.value()] = gtsam::Pose3{rotation, translation};
        }

        return worldTtags;
    }

    float width = 6.5 * 25.4 / 1000.0; // 6.5in wide tag
    vector<Point3> tagToCorners{
        {0, -width / 2.0, -width / 2.0},
        {0, width / 2.0, -width / 2.0},
        {0, width / 2.0, width / 2.0},
        {0, -width / 2.0, width / 2.0},
    };
}

namespace tagmodel
{
    vector<Point3_> WorldToCornersFactor(Pose3_ worldTtag)
    {
        vector<Point3_> out;
        for (const auto &p : tagToCorners)
        {
            out.push_back(transformFrom(worldTtag, p));
        }

        return out;
    }

    std::optional<vector<Point3>> WorldToCorners(int id, nlohmann::json json)
    {
        auto worldTtags = TagLayoutToMap(json);
        auto maybePose = worldTtags.find(id);
        if (maybePose == worldTtags.end())
        {
            return std::nullopt;
        }
        Pose3 worldTtag = maybePose->second;

        vector<Point3> out(4);
        std::transform(
            tagToCorners.begin(), tagToCorners.end(), out.begin(),
            [&worldTtag](const auto &p)
            { return worldTtag.transformFrom(p); });

        return out;
    }

    std::optional<gtsam::Pose3> worldToTag(int id, nlohmann::json json)
    {
        auto worldTtags = TagLayoutToMap(json);
        if (auto it = worldTtags.find(id); it != worldTtags.end())
        {
            return it->second;
        }
        else
        {
            return std::nullopt;
        }
    }
}
