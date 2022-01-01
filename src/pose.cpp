//
// Created by Megan Finch on 29/12/2021.
//

#include <headers/pose.h>
#include <opencv2/core/matx.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>


Pose::Pose(const cv::Matx34d& mat) : _mat(mat) {}

Pose::Pose(const cv::Matx31d& rvec, const cv::Matx31d& tvec) {
    cv::Matx33d rmat;
    cv::Rodrigues(rvec, rmat);

    // TODO: verify correctness
    _mat = cv::Matx34d(rmat(0,0), rmat(0,1), rmat(0,2), tvec(0,0),
                       rmat(1,0), rmat(1,1), rmat(1,2), tvec(1,0),
                       rmat(2,0), rmat(2,1), rmat(2,2), tvec(2,0));
}

cv::Matx34d Pose::getProjectionMatrix() {
    return _mat;
}

cv::Matx31d Pose::getRotationVector() const {
    cv::Mat rvec;
    cv::Rodrigues(_mat.get_minor<3,3>(0,0), rvec);
    cv::Matx31d result(rvec.at<double>(0,0), rvec.at<double>(1,0), rvec.at<double>(2,0));
    return result;
}

cv::Matx13d Pose::getTranslationVector() const {
    cv::Mat tvec(_mat.get_minor<3,1>(0,3).t());
    cv::Matx13d result(tvec.at<double>(0,0), tvec.at<double>(0,1), tvec.at<double>(0,2));
    return result;
}
