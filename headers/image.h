//
// Created by Megan Finch on 23/12/2021.
//

#ifndef SFM_IMAGE_H
#define SFM_IMAGE_H

#include "headers/types.h"
#include <string>

#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>

class Image {
private:
    ImageID id;
    std::string name;
    cv::Mat data;

public:
    Image(std::string filepath);
};

#endif //SFM_IMAGE_H
