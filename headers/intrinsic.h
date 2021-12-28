//
// Created by Megan Finch on 23/12/2021.
//

#ifndef SFM_CAMERA_H
#define SFM_CAMERA_H

#include <iostream>
#include "image.h"

class Intrinsic {
private:
    cv::Mat_<double> distortion;
    cv::Matx33d K;

public:
    explicit Intrinsic(const Image& image);

};

#endif //SFM_CAMERA_H
