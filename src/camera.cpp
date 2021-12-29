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

double Camera::getFocalLength() {
    return K(0,0);
}

cv::Point2d Camera::getCentre() {
    return cv::Point2d(K(0,2), K(1,2));
}
