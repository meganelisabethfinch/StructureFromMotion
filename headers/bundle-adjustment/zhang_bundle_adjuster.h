//
// Created by Megan Finch on 24/02/2022.
//

#ifndef SFM_ZHANG_BUNDLE_ADJUSTER_H
#define SFM_ZHANG_BUNDLE_ADJUSTER_H

#include "bundle_adjuster.h"

class ZhangBundleAdjuster : public BundleAdjuster {
    void adjustBundle(PointCloud& pointCloud,
                              const std::set<ImageID>& registeredImages,
                              std::map<ImageID, Pose>& cameraPoses,
                              std::vector<Camera>& cameras,
                              std::vector<Features>& features) override
    {
        // TODO: implement Zhang RBA (2006).
    };
};
#endif //SFM_ZHANG_BUNDLE_ADJUSTER_H
