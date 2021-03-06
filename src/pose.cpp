//
// Created by Megan Finch on 29/12/2021.
//

#include <headers/pose.h>
#include <opencv2/core/matx.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <ceres/rotation.h>


Pose::Pose(const cv::Matx34d& mat) : _mat(mat) {}

Pose::Pose(const PoseVector& pose) {
    // Convert angle-axis back to rotation matrix
    double rmat[9] = { 0 };
    ceres::AngleAxisToRotationMatrix(pose.val, rmat);

    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            _mat(c,r) = rmat[r * 3 + c]; // rmat is col-major
        }
    }

    // Translation
    _mat(0,3) = pose(3);
    _mat(1,3) = pose(4);
    _mat(2,3) = pose(5);
}

Pose::Pose(const cv::Matx31d& rvec, const cv::Matx31d& tvec) {
    cv::Matx33d rmat;
    cv::Rodrigues(rvec, rmat);

    // TODO: verify correctness
    _mat = cv::Matx34d(rmat(0,0), rmat(0,1), rmat(0,2), tvec(0,0),
                       rmat(1,0), rmat(1,1), rmat(1,2), tvec(1,0),
                       rmat(2,0), rmat(2,1), rmat(2,2), tvec(2,0));
}

cv::Matx34d Pose::getProjectionMatrix() const {
    return _mat;
}

cv::Matx31d Pose::getRotationVector() const {
    cv::Mat rvec;
    cv::Rodrigues(_mat.get_minor<3,3>(0,0), rvec);
    return { rvec };
}

cv::Matx13d Pose::getTranslationVectorAlt() const {
    cv::Mat tvec(_mat.get_minor<3,1>(0,3).t());
    return { tvec };
}

cv::Matx31d Pose::getTranslationVector() const {
    return { _mat(0, 3), _mat(1, 3), _mat(2,3) };
}

PoseVector Pose::toPoseVector() const {
    cv::Vec3d t(_mat(0,3), _mat(1,3), _mat(2,3));
    cv::Matx33d R = _mat.get_minor<3,3>(0,0);

    double angleAxis[3];
    ceres::RotationMatrixToAngleAxis<double>(R.t().val, angleAxis);

    return {
            angleAxis[0], angleAxis[1], angleAxis[2],
            t(0), t(1), t(2)
    };
}

cv::Matx33d Pose::getRotationMatrix() const {
    return { _mat(0,0), _mat(0,1), _mat(0,2),
             _mat(1,0), _mat(1,1), _mat(1,2),
             _mat(2,0), _mat(2,1), _mat(2,2) };
}
