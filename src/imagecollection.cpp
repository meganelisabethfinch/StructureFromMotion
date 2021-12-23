//
// Created by Megan Finch on 23/12/2021.
//

#include <headers/imagecollection.h>
#include <string>

#include <opencv2/imgcodecs.hpp>
#include <headers/image.h>

ImageCollection::ImageCollection(std::string directory) {
    std::vector<cv::String> filenames;
    cv::glob(directory + "/*.png", filenames, false);

    for (const auto& fn : filenames) {
        images.emplace_back(Image(fn));
    }
}