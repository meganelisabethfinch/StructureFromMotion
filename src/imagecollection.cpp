//
// Created by Megan Finch on 23/12/2021.
//

#include <headers/imagecollection.h>

#include <string>
#include <filesystem>
#include <iostream>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <headers/sfm_util.h>

ImageCollection::ImageCollection(const std::string& directory) {
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
        auto camera = Camera(image);
        mCameras.push_back(camera);
    }

    mFeatureMatchMatrix = Matches();
}

void ImageCollection::ExtractFeatures(const cv::Ptr<cv::FeatureDetector>& detector) {
    mImageFeatures.clear();

    for (auto & image : mImages) {
        mImageFeatures.emplace_back(Features(detector, image));
        std::cout << "Image " << image.id << ": " << mImageFeatures.back().size() << " keypoints" << std::endl;
    }
}

bool ImageCollection::FindMatches(const cv::Ptr<cv::DescriptorMatcher> &matcher) {
    if (mImageFeatures.size() != mImages.size())
    {
        std::cout << "Features must be initialised before matches are found" << std::endl;
        return false;
    }

    if (mImageFeatures.size() < 2) {
        std::cout << "At least two sets of features are required to find matches" << std::endl;
        return false;
    }

    mFeatureMatchMatrix = Matches(matcher, mImageFeatures);

    return true;
}

size_t ImageCollection::size() const {
    return mImages.size();
}

Image &ImageCollection::getImage(ImageID idx) {
    return mImages[idx];
}

void ImageCollection::visualiseKeyPoints(ImageID idx) {
    cv::Mat out;
    std::vector<cv::KeyPoint> keypoints = mImageFeatures[idx].getCVKeyPoints();
    cv::drawKeypoints(mImages[idx].data, keypoints, out, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow("Keypoints", out);
    cv::waitKey(0);
}

void ImageCollection::visualiseMatches(ImageID i, ImageID j) {
    cv::Mat out;
    std::vector<cv::KeyPoint> iKeypoints = mImageFeatures[i].getCVKeyPoints();
    std::vector<cv::KeyPoint> jKeypoints = mImageFeatures[j].getCVKeyPoints();
    Matching2 matches = mFeatureMatchMatrix.getMatchingBetween(i, j);
    cv::drawMatches(mImages[i].data, iKeypoints, mImages[j].data, jKeypoints, matches, out);
    cv::imshow("Matches", out);
    cv::waitKey(0);
}

SceneReconstruction ImageCollection::toSceneReconstruction(ImagePair& imagePair) {
    auto recon = SceneReconstruction(mImages, mCameras, mImageFeatures, mFeatureMatchMatrix, imagePair);
    // recon.adjustBundle(); -- seems to make it worse?

    for (ImageID i = 0; i < mImages.size(); i++) {
        if (recon.registerImage(i) && i % 4 == 0) {
            recon.adjustBundle();
        }
    }

    return recon;
}

SceneReconstruction ImageCollection::toSceneReconstruction() {
    auto recon = SceneReconstruction(mImages, mCameras, mImageFeatures, mFeatureMatchMatrix);

    return recon;
}

SceneGraph ImageCollection::toSceneGraph() {
    return { mImages, mFeatureMatchMatrix };
}

ImageCollection::ImageCollection() = default;
