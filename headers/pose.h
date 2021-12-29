//
// Created by Megan Finch on 29/12/2021.
//

#ifndef SFM_POSE_H
#define SFM_POSE_H

#include <opencv2/core/matx.hpp>

class Pose {
private:
    cv::Matx34d _mat;

public:
    explicit Pose(const cv::Matx34d& mat);

    cv::Matx34d getProjectionMatrix();

    // get Rotation vector

    // get Rotation as Quaternion
};

#endif //SFM_POSE_H
