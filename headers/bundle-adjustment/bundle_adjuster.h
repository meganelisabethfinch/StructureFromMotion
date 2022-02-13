//
// Created by Megan Finch on 13/02/2022.
//

#ifndef SFM_BUNDLE_ADJUSTER_H
#define SFM_BUNDLE_ADJUSTER_H

class BundleAdjuster {
    virtual void adjustBundle(PointCloud& pointCloud,
            const std::set<ImageID>& registeredImages,
            std::map<ImageID, Pose>& cameraPoses,
            std::vector<Camera>& cameras,
            std::vector<Features>& features) = 0;
};

#endif //SFM_BUNDLE_ADJUSTER_H
