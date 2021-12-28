//
// Created by Megan Finch on 23/12/2021.
//

#include <headers/imagecollection.h>

#include <string>
#include <filesystem>
#include <iostream>

#include <opencv2/imgcodecs.hpp>

ImageCollection::ImageCollection(std::string directory) {
    std::vector<cv::String> filenames;
    cv::glob(directory + "/*.png", filenames, false);

    for (const auto& fn : filenames) {
        cv::Mat data = cv::imread(fn, cv::IMREAD_COLOR);

        if (data.empty()) {
            std::cout << "Unable to read image from file: " << fn << std::endl;
            continue;
        }

        std::string name = std::filesystem::path(fn).filename();
        ImageID id = images.size();
        Image image = Image(id, name, data);
        images.push_back(image);

        // Initialise intrinsic/camera matrix
        Intrinsic camera = Intrinsic(image);
        intrinsics.push_back(camera);
    }
}
