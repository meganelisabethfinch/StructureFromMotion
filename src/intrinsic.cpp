//
// Created by Megan Finch on 28/12/2021.
//

#include <headers/intrinsic.h>
#include <headers/image.h>
#include <headers/constants.h>

Intrinsic::Intrinsic(const Image& image) {
    this->K = cv::Matx<double, 3, 3>{
            FOCAL_LENGTH, 0, (double)(image.data.cols / 2),
            0, FOCAL_LENGTH, (double)(image.data.rows / 2),
            0, 0, 1};

    this->distortion = cv::Mat_<double>::zeros(1, 4);
};
