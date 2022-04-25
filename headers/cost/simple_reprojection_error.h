//
// Created by Megan Finch on 07/01/2022.
//

#ifndef SFM_SIMPLE_REPROJECTION_ERROR_H
#define SFM_SIMPLE_REPROJECTION_ERROR_H

#include <ceres/rotation.h>
#include <ceres/cost_function.h>
#include <ceres/autodiff_cost_function.h>

struct SimpleReprojectionError {
    double observed_x;
    double observed_y;

    SimpleReprojectionError(double x, double y)
        : observed_x(x), observed_y(y) {}

    /*
     * @param pose
     * @param point
     * @param focal
     * @param residuals - the computed error is written back to the residuals array
     * @return true to indicate success
     */
    template<typename T>
    bool operator()(const T* const pose,
            const T* const point,
            const T* const focal,
            T* residuals) const {
        T p[3];

        // Rotate: pose[0,1,2] are the angle-axis rotation
        ceres::AngleAxisRotatePoint(pose, point, p);

        // Translate: pose[3,4,5] are the translation
        p[0] += pose[3];
        p[1] += pose[4];
        p[2] += pose[5];

        // Perspective divide
        const T xp = p[0] / p[2];
        const T yp = p[1] / p[2];

        // Compute final projected point
        const T predicted_x = *focal * xp;
        const T predicted_y = *focal * yp;

        residuals[0] = predicted_x - T(observed_x);
        residuals[1] = predicted_y - T(observed_y);
        return true;
    }

    static ceres::CostFunction* Create(const double observed_x, const double observed_y) {
        // AutoDiffCostFunction<CostFunctor, num of residuals, num of dimensions for each parameter>
        // residuals - 2 (for x and y)
        // pose - 6 (3 for rotation, 3 for translation)
        // point3d - 3 (for x, y, and z)
        // focal - 1 (assuming focal_x = focal_y)
        return (new ceres::AutoDiffCostFunction<SimpleReprojectionError, 2, 6, 3, 1>(
                new SimpleReprojectionError(observed_x, observed_y)));
    }
};

#endif //SFM_SIMPLE_REPROJECTION_ERROR_H
