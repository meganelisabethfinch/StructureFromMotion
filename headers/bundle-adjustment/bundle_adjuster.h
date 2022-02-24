//
// Created by Megan Finch on 13/02/2022.
//

#ifndef SFM_BUNDLE_ADJUSTER_H
#define SFM_BUNDLE_ADJUSTER_H

#include "../point_cloud.h"
#include "../pose.h"
#include "../camera.h"

class BundleAdjuster {
public:
    virtual void adjustBundle(PointCloud& pointCloud,
            const std::set<ImageID>& registeredImages,
            std::map<ImageID, Pose>& cameraPoses,
            std::vector<Camera>& cameras,
            std::vector<Features>& features) = 0;

    static cv::Ptr<BundleAdjuster> create(const BundleAdjusterType& type);
};

#endif //SFM_BUNDLE_ADJUSTER_H
