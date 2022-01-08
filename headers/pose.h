//
// Created by Megan Finch on 29/12/2021.
//

#ifndef SFM_POSE_H
#define SFM_POSE_H

#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include "types.h"

class Pose {
private:

    cv::Matx34d _mat;

public:
    explicit Pose(const cv::Matx34d& mat);

    explicit Pose(const PoseVector& pose);

    Pose(const cv::Matx31d& rvec, const cv::Matx31d& tvec);

    cv::Matx34d getProjectionMatrix();

    [[nodiscard]] cv::Matx31d getRotationVector() const;

    [[nodiscard]] cv::Matx13d getTranslationVector() const;

    // get Rotation as Quaternion
};

#endif //SFM_POSE_H
