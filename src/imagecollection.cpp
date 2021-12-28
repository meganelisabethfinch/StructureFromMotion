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
        ImageID id = mImages.size();
        Image image = Image(id, name, data);
        mImages.push_back(image);

        // Initialise intrinsic/camera matrix
        Intrinsic camera = Intrinsic(image);
        mIntrinsics.push_back(camera);
    }

    mFeatureMatchMatrix = Matches();
}

void ImageCollection::ExtractFeatures(const cv::Ptr<cv::FeatureDetector>& detector) {
    mImageFeatures.clear();

    for (auto & image : mImages) {
        mImageFeatures.emplace_back(Features(detector, image));
    }
}

bool ImageCollection::FindMatches(const cv::Ptr<cv::DescriptorMatcher> &matcher) {
    if (mImageFeatures.size() != mImages.size())
    {
        std::cout << "Features must be initialised before matches are found." << std::endl;
        return false;
    }

    if (mImageFeatures.size() < 2) {
        std::cout << "At least two sets of features are required to find matches." << std::endl;
        return false;
    }

    mFeatureMatchMatrix = Matches(matcher, mImageFeatures);

    return true;
}

size_t ImageCollection::size() const {
    return mImages.size();
}


