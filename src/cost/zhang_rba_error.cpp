//
// Created by Megan Finch on 09/01/2022.
//
#include <headers/cost/zhang_rba_error.h>
#include <opencv2/core/types.hpp>
#include <utility>
#include <ceres/autodiff_cost_function.h>

ZhangRBAError::ZhangRBAError(const std::array<std::array<cv::Point2d, 6>, 3>& points2d) {
    // Calculate k1, k2, and k3
    for (size_t j = 0; j < 3; j++) {
        for (size_t i = 0; i < 6; i++) {
            auto pt1 = cv::Vec3d(points2d.at(j).at(i).x, points2d.at(j).at(i).y, 1.0);
            auto pt2 = cv::Vec3d(points2d.at(j).at(0).x, points2d.at(j).at(0).y, 1.0);
            auto pt3 = cv::Vec3d(points2d.at(j).at(1).x, points2d.at(j).at(1).y, 1.0);

            k1[j][i] = pt1.dot(pt2);
            k2[j][i] = (pt2.cross(pt1)).dot(pt2.cross(pt3));
            k3[j][i] = pt1.dot(pt2.cross(pt3));
        }
    }
}

template <typename T>
bool ZhangRBAError::operator()(const T* const points3d,
                const T* const projectionCentres,
                const T* const gammas,
                T* residuals) const {
    // TODO: See equation 4
    T error;

    for (size_t j = 0; j < 3; j++) {
        for (size_t i = 0; i < 6; i++) {
            // auto Pi = cv::Vec3d(points3d[3*i], points3d[3*i+1], points3d[3*i+2]);

        }
    }

    residuals[0] = error;
    return true;
}

ceres::CostFunction* ZhangRBAError::Create(const std::array<std::array<cv::Point2d, 6>, 3>& points2d) {
    // num residuals - 3 (or just 1?)
    // points3d - 18, [x,y,z...] for each of 6 points
    // projectionCentres - 6, one [x, y] for each image
    // gammas - 18, one for each image * point pair
    return (new ceres::AutoDiffCostFunction<ZhangRBAError, 3, 18, 6, 18>(
            new ZhangRBAError(points2d)
    ));
}
