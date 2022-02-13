//
// Created by Megan Finch on 07/01/2022.
//

#ifndef SFM_BA_UTIL_H
#define SFM_BA_UTIL_H

#include "point_cloud.h"
#include "pose.h"
#include "camera.h"

#include <set>

class BundleAdjustmentUtilities {
private:

public:
    static void adjustBundle(PointCloud& pointCloud,
                      const std::set<ImageID>& registeredImages,
                      std::map<ImageID, Pose>& cameraPoses,
                      std::vector<Camera>& cameras,
                      std::vector<Features>& features);

};

#endif //SFM_BA_UTIL_H
