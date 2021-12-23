//
// Created by Megan Finch on 23/12/2021.
//

#include <headers/image.h>

#include <opencv2/imgcodecs.hpp>

#include <string>
#include <filesystem>

Image::Image(std::string filepath) {
    this->name = std::filesystem::path(filepath).filename();
    this->data = cv::imread(filepath, cv::IMREAD_COLOR);
}
