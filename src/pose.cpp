//
// Created by Megan Finch on 29/12/2021.
//

#include <headers/pose.h>
#include <opencv2/core/matx.hpp>


Pose::Pose(const cv::Matx34d& mat) : _mat(mat) {}

cv::Matx34d Pose::getPoseMatrix() {
    return _mat;
}

