#include <headers/cost/faster_rotation_free_error.h>
#include <opencv2/core/types.hpp>
#include <utility>
#include <ceres/autodiff_cost_function.h>


FasterRotationFreeError::FasterRotationFreeError(const std::array<std::array<cv::Point2d, 6>, 2>& points2d) {
    // Calculate k1, k2, and k3 - same as RotationFreeError
    for (size_t j = 0; j < 6; j++) {
        for (size_t i = 0; i < 2; i++) {
            // (p_ij, 1)
            auto pt1 = cv::Vec3d(points2d.at(i).at(j).x, points2d.at(i).at(j).y, 1.0);
            // (p_1j, 1)
            auto pt2 = cv::Vec3d(points2d.at(0).at(j).x, points2d.at(0).at(j).y, 1.0);
            // (p_2j, 1)
            auto pt3 = cv::Vec3d(points2d.at(1).at(j).x, points2d.at(1).at(j).y, 1.0);

            k1[i][j] = pt1.dot(pt2);
            k2[i][j] = (pt2.cross(pt1)).dot(pt2.cross(pt3));
        }
    }
}

template <typename T>
bool FasterRotationFreeError::operator()(const T* const points3d,
                                   const T* const centres,
                                   const T* const gammas,
                                   T* residuals) const {
    // See equation (4) of Zhang et al. RBA
    /*
    T total = 0;

    auto P1 = cv::Vec3d(points3d[0], points3d[1], points3d[2]);
    auto P2 = cv::Vec3d(points3d[3], points3d[4], points3d[5]);

    for (size_t j = 0; j < 6; j++) {
        auto Cj = cv::Vec3d(centres[3*j], centres[3*j+1], centres[3*j+2]);

        auto v1 = P1 - Cj;
        auto v2 = P2 - Cj;

        auto t1 = v1.dot(v1) - gammas[j] * gammas[j] * k1[0][j];
        auto t2 = v2.dot(v2) - gammas[6 + j] * gammas[6 + j] * k2[1][j];
        auto t3 = v2.dot(v1) - gammas[6 + j] * gammas[j] * k1[1][j];

        // Square each subterm and add to total
        auto term = t1*t1 + t2*t2 + t3*t3;
        total += term;
    }

    residuals[0] = total;
     */
    return true;
}

ceres::CostFunction* FasterRotationFreeError::Create(const std::array<std::array<cv::Point2d, 6>, 2>& points2d) {
    // num residuals - 1
    // points3d - 6, [x,y,z] for each of 2 points
    // centres - 18, one [x, y, z] for each of 6 images
    // gammas - 12, one for each image * point pair
    return (new ceres::AutoDiffCostFunction<FasterRotationFreeError, 1, 6, 18, 12>(
            new FasterRotationFreeError(points2d)
    ));
}
