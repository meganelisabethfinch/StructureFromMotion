#include <headers/cost/faster_rotation_free_error.h>
#include <opencv2/core/types.hpp>
#include <utility>
#include <ceres/autodiff_cost_function.h>
#include <headers/vector_util.h>


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
    // See equation (5) of Zhang et al. RBA
    T terms[6];

    for (size_t j = 0; j < 6; j++) {
        T v1[3], v2[3];
        // v1 = P1 - Cj
        VectorUtilities::subtract(points3d, centres + 3*j, 3, v1);
        // v2 = P2 - Cj
        VectorUtilities::subtract(points3d + 3, centres + 3*j, 3, v2);

        // Calculate dot products
        T d1[1], d2[1], d3[1];
        // d1 = |P1 - Cj|^2 = v1 . v1
        VectorUtilities::dot(v1, v1, 3, d1);
        // d2 = |P2 - Cj|^2 = v2 . v2
        VectorUtilities::dot(v2, v2, 3, d2);
        // d3 = (P2 - Cj).(P1 - Cj) = v2 . v1
        VectorUtilities::dot(v2, v1, 3, d3);

        // gammas[3*i + j] = gamma_ij
        T t1 = d1[0] - gammas[j] * gammas[j] * k1[0][j];
        T t2 = d2[0] - gammas[3 + j] * gammas[3 + j] * k2[1][j];
        T t3 = d3[0] - gammas[3 + j] * gammas[j] * k1[1][j];

        // Square and sum each subterm
        terms[j] = t1*t1 + t2*t2 + t3*t3;
    }

    T total = terms[0];
    for (size_t j = 1; j < 6; j++) {
        total += terms[j];
    }

    residuals[0] = total;
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
