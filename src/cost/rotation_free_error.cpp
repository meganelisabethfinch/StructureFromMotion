//
// Created by Megan Finch on 09/01/2022.
//
#include <headers/cost/rotation_free_error.h>
#include <opencv2/core/types.hpp>
#include <utility>
#include <ceres/autodiff_cost_function.h>
#include <headers/vector_util.h>

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
    T terms[18];

    // Calculate each term
    for (size_t j = 0; j < 3; j++) {
        for (size_t i = 0; i < 6; i++) {
            T v1[3], v2[3], v3[3];
            // v1 = Pi - Cj
            VectorUtilities::subtract(points3d + 3*i, centres + 3*j, 3, v1);
            // v2 = P1 - Cj
            VectorUtilities::subtract(points3d, centres + 3*j, 3, v2);
            // v3 = P2 - Cj
            VectorUtilities::subtract(points3d + 3, centres + 3*j, 3, v3);

            // Calculate cross products
            T v4[3], v5[3];
            // v4 = (P1 - Cj) x (Pi - Cj) = v2 x v1
            VectorUtilities::cross(v2, v1, v4);
            // v5 = (P1 - Cj) x (P2 - Cj) = v2 x v3
            VectorUtilities::cross(v2, v3, v5);

            // Calculate dot products
            T d1[1], d2[1], d3[1];
            // d1 = (Pi - Cj) . (P1 - Cj) = v1 . v2
            VectorUtilities::dot(v1, v2, 3, d1);
            // d2 = (P1 - Cj) x (Pi - Cj) . (P1 - Cj) x (P2 - Cj) = v4 . v5
            VectorUtilities::dot(v4, v5, 3, d2);
            // d3 = (Pi - Cj) . (P1 - Cj) x (P2 - Cj) = v1 . v5
            VectorUtilities::dot(v1, v5, 3, d3);

            // gammas[3*i +j] = gamma_ij
            T t1 = d1[0] - gammas[3*i + j] * gammas[j] + k1[i][j];
            T t2 = d2[0] - gammas[3*i + j] * gammas[j] * gammas[j] * gammas[3 + j] * k2[i][j];
            T t3 = d3[0] - gammas[3*i + j] * gammas[j] * gammas[3 + j] * k3[i][j];

            // Square and sum each subterm
            terms[3*i + j] = t1*t1 + t2*t2 + t3*t3;
        }
    }

    // Sum terms
    T total = terms[0];

    for (size_t j = 0; j < 3; j++) {
        for (size_t i = 0; i < 6; i++) {
            if (i == 0 & j == 0) {
                // skip term used to intialise accumulator
                continue;
            } else {
                total += terms[3*i + j];
            }
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
