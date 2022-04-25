//
// Created by Megan Finch on 07/01/2022.
//

#ifndef SFM_BA_UTIL_H
#define SFM_BA_UTIL_H

#include "point_cloud.h"
#include "pose.h"
#include "camera.h"

#include <set>

class BAUtilities {
private:

public:
    /*
     * Quick and hacky way to compute total reprojection error using ceres solver.
     * Copy of the code from BasicBundleAdjuster, with the max iterations set to 0.
     */
    static double globalReprojectionError(PointCloud& pc,
                                          const std::set<ImageID>& registeredImages,
                                          std::map<ImageID, Pose>& cameraPoses,
                                          std::vector<Camera>& cameras,
                                          std::vector<Features>& features,
                                          LossType lossType);

};

#endif //SFM_BA_UTIL_H
