//
// Created by Megan Finch on 08/01/2022.
//

#ifndef SFM_ZHANG_RBA_ERROR_H
#define SFM_ZHANG_RBA_ERROR_H

#include <array>
#include <opencv2/core/types.hpp>
#include <ceres/cost_function.h>
#include <set>
#include <headers/types.h>
#include <headers/point_cloud.h>

// Rotation-free cost function as described in
// "Robust Bundle Adjustment for Structure from Motion" (Zhang et. al) (2006)

class RotationFreeError {
    double k1[6][3]{};
    double k2[6][3]{};
    double k3[6][3]{};

public:
    /*
     * @param points2d - takes a tuple of 18 image points, indexed by points2d[i][j],
     *      corresponding to a sextuplet of 3d points seen by 3 views.
     *      The points are indexed i=0...5 and the views j=0...2
     */
    explicit RotationFreeError(const std::array<std::array<cv::Point2d, 3>, 6>& points2d);

    template <typename T>
    bool operator()(const T* points3d,
            const T* centres,
            const T* gammas,
            T* residuals) const;

    static ceres::CostFunction* Create(const std::array<std::array<cv::Point2d, 3>, 6>& points3d);
};

#endif //SFM_ZHANG_RBA_ERROR_H
