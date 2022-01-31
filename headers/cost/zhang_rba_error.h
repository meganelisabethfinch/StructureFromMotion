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

class ZhangRBAError {
    double k1[3][6]{};
    double k2[3][6]{};
    double k3[3][6]{};

public:
    explicit ZhangRBAError(const std::array<std::array<cv::Point2d, 6>, 3>& points3d);

    template <typename T>
    bool operator()(const T* const points3d,
            const T* const projectionCentres,
            const T* const gammas,
            T* residuals) const;

    static ceres::CostFunction* Create(const std::array<std::array<cv::Point2d, 6>, 3>& points3d);

    static std::set<double> GetPointsWithCommonViews(PointCloud &pointCloud, size_t N, size_t J);

    static void CommonViewsHelper(PointCloud &pointCloud, size_t pointsNeeded, size_t viewsNeeded,
         std::set<ImageID> &commonViews, std::vector<int> &points, std::set<double> &tuples);
};

#endif //SFM_ZHANG_RBA_ERROR_H
