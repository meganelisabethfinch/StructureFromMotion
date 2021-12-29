//
// Created by Megan Finch on 23/12/2021.
//

#ifndef SFM_CAMERA_H
#define SFM_CAMERA_H

#include <iostream>
#include "image.h"

class Camera {
private:
    cv::Mat_<double> distortion;
    cv::Matx33d K;

public:
    explicit Camera(const Image& image);

    cv::Matx33d getIntrinsicMatrix() const;
};

#endif //SFM_CAMERA_H
