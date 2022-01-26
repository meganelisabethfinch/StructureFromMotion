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
    static void _CommonViewsHelper(PointCloud& pointCloud,
                                   size_t pointsNeeded,
                                   size_t viewsNeeded,
                                   std::set<ImageID>& commonViews,
                                   std::vector<int>& points,
                                   std::set<double>& tuples);

public:
    static void adjustBundle(PointCloud& pointCloud,
                      const std::vector<Image>& images,
                      std::map<ImageID, Pose>& cameraPoses,
                      std::vector<Camera>& cameras,
                      std::vector<Features>& features);

    /*
     * @param pointCloud
     * @param N number of points
     * @param J number of common images
     * @return the set of all tuples of N points in which each point appears in the same J images
     */
    static std::set<double> GetPointsWithCommonViews(PointCloud& pointCloud, size_t N, size_t J);
};

#endif //SFM_BA_UTIL_H
