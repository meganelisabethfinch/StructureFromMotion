//
// Created by Megan Finch on 28/12/2021.
//

#include <headers/camera.h>
#include <headers/image.h>
#include <headers/constants.h>

Camera::Camera(const Image& image) {
    this->K = cv::Matx<double, 3, 3>{
            FOCAL_LENGTH, 0, (double)(image.data.cols / 2),
            0, FOCAL_LENGTH, (double)(image.data.rows / 2),
            0, 0, 1};

    this->distortion = cv::Mat_<double>::zeros(1, 4);
}

cv::Matx33d Camera::getCameraMatrix() const {
    return K;
};

double Camera::getFocalLength() const {
    return K(0,0);
}

cv::Point2d Camera::getCentre() const {
    return {K(0,2), K(1,2)};
}

cv::Mat_<double> Camera::getDistortion() {
    return distortion;
}

void Camera::setFocalLength(double fx, double fy) {
    K(0,0) = fx;
    K(1,1) = fy;
}
