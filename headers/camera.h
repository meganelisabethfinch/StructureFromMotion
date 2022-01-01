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

    [[nodiscard]] cv::Matx33d getCameraMatrix() const;

    [[nodiscard]] double getFocalLength() const;

    [[nodiscard]] cv::Point2d getCentre() const;

    cv::Mat_<double> getDistortion();
};

#endif //SFM_CAMERA_H
