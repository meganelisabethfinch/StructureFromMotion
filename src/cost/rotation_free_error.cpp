//
// Created by Megan Finch on 09/01/2022.
//
#include <headers/cost/rotation_free_error.h>
#include <opencv2/core/types.hpp>
#include <utility>
#include <ceres/autodiff_cost_function.h>

RotationFreeError::RotationFreeError(const std::array<std::array<cv::Point2d, 3>, 6>& points2d) {
    // Calculate k1, k2, and k3
    for (size_t j = 0; j < 3; j++) {
        for (size_t i = 0; i < 6; i++) {
            // (p_ij, 1)
            auto pt1 = cv::Vec3d(points2d.at(i).at(j).x, points2d.at(i).at(j).y, 1.0);
            // (p_1j, 1)
            auto pt2 = cv::Vec3d(points2d.at(0).at(j).x, points2d.at(0).at(j).y, 1.0);
            // (p_2j, 1)
            auto pt3 = cv::Vec3d(points2d.at(1).at(j).x, points2d.at(1).at(j).y, 1.0);

            k1[i][j] = pt1.dot(pt2);
            k2[i][j] = (pt2.cross(pt1)).dot(pt2.cross(pt3));
            k3[i][j] = pt1.dot(pt2.cross(pt3));
        }
    }
}

template <typename T>
bool RotationFreeError::operator()(const T* const points3d,
                const T* const centres,
                const T* const gammas,
                T* residuals) const {
    // See equation (4) of Zhang et al. RBA
    double total = 0;

    auto P1 = cv::Vec3d(points3d[0], points3d[1], points3d[2]);
    auto P2 = cv::Vec3d(points3d[3], points3d[4], points3d[5]);

    for (size_t j = 0; j < 3; j++) {
        auto Cj = cv::Vec3d(centres[3*j], centres[3*j+1], centres[3*j+2]);

        for (size_t i = 0; i < 6; i++) {
            auto Pi = cv::Vec3d(points3d[3*i], points3d[3*i+1], points3d[3*i+2]);

            auto t1 = (Pi - Cj).dot(P1 - Cj)
                    - gammas[6*i + j] * gammas[j] * k1[i][j];
            auto t2 = ((P1 - Cj).cross(Pi - Cj)).dot((P1 - Cj).cross(P2 - Cj))
                    - gammas[6*i + j] * (gammas[j]*gammas[j]) * gammas[6 + j] * k2[i][j];
            auto t3 = (Pi - Cj).dot((P1 - Cj).cross(P2 - Cj))
                    - gammas[6*i + j] * gammas[j] * gammas[6 + j] * k3[i][j];

            // Square each subterm and add to total
            auto term = t1*t1 + t2*t2 + t3*t3;
            total += term;
        }
    }

    residuals[0] = total;
    return true;
}

ceres::CostFunction* RotationFreeError::Create(const std::array<std::array<cv::Point2d, 3>, 6>& points2d) {
    // num residuals - 1
    // points3d - 18, [x,y,z] for each of 6 points
    // centres - 9, one [x, y, z] for each of 3 images
    // gammas - 18, one for each image * point pair
    return (new ceres::AutoDiffCostFunction<RotationFreeError, 1, 18, 9, 18>(
            new RotationFreeError(points2d)
    ));
}
