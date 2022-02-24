//
// Created by Megan Finch on 24/02/2022.
//

#ifndef SFM_DUMMY_BUNDLE_ADJUSTER_H
#define SFM_DUMMY_BUNDLE_ADJUSTER_H

#include "bundle_adjuster.h"

// Because sometimes less is more
class DummyBundleAdjuster : public BundleAdjuster {
    void adjustBundle(PointCloud& pointCloud,
                              const std::set<ImageID>& registeredImages,
                              std::map<ImageID, Pose>& cameraPoses,
                              std::vector<Camera>& cameras,
                              std::vector<Features>& features) override
    {
        // Do literally nothing.
    };
};

#endif //SFM_DUMMY_BUNDLE_ADJUSTER_H
