
#ifndef SFM_FASTER_ROTATION_FREE_ERROR_H
#define SFM_FASTER_ROTATION_FREE_ERROR_H

#include <array>
#include <ceres/cost_function.h>
#include <opencv2/core/types.hpp>

class FasterRotationFreeError {
// k constants indexed [i][j] where i=0..1 (3d points) and j=0..5 (images)
    double k1[2][6]{};
    double k2[2][6]{};
    double k3[2][6]{};
public:
    /*
    * @param points2d - takes a tuple of 18 image points, indexed by points2d[i][j],
    *      corresponding to a pair of 3d points seen by 6 views.
    *      The points are indexed i=0..1 and the views j=0..5
    */
    explicit FasterRotationFreeError(const std::array<std::array<cv::Point2d, 6>, 2>& points2d);

    template <typename T>
    bool operator()(const T* points3d,
                    const T* centres,
                    const T* gammas,
                    T* residuals) const;

    static ceres::CostFunction* Create(const std::array<std::array<cv::Point2d, 6>, 2>& points2d);
};

#endif //SFM_FASTER_ROTATION_FREE_ERROR_H
