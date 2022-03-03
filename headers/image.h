//
// Created by Megan Finch on 28/12/2021.
//

#ifndef SFM_IMAGE_H
#define SFM_IMAGE_H

#include <utility>

#include "types.h"

#include <opencv2/core/mat.hpp>

struct Image {
    ImageID id;
    std::string name;
    cv::Mat data;
    Image(ImageID imageId, std::string imageName, cv::Mat& imageData)
        : id(imageId), name(std::move(imageName)), data(imageData) {}

    [[nodiscard]] cv::Vec3b getColourAt(const cv::Point2d& point) const {
        return data.at<cv::Vec3b>(point);
    }
};

#endif //SFM_IMAGE_H
