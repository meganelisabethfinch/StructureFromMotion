//
// Created by Megan Finch on 28/12/2021.
//

#include <headers/features.h>
#include <headers/image.h>
#include <headers/sfm_util.h>

Features::Features(const cv::Ptr<cv::FeatureDetector>& detector, const Image& image) {
    detector->detectAndCompute(image.data, cv::noArray(), _keypoints, _descriptors);

    for (const auto& kp : _keypoints) {
        _points2d.push_back(kp.pt);
    }
}

std::vector<cv::KeyPoint>& Features::getCVKeyPoints() {
    return _keypoints;
}

cv::Mat& Features::getCVDescriptors() {
    return _descriptors;
}

Matching2& Features::FindMatchesWith(const cv::Ptr<cv::DescriptorMatcher> &matcher, Features &other) {
    std::vector<std::vector<cv::DMatch>> initialMatching;
    matcher->knnMatch(_descriptors, other.getCVDescriptors(), initialMatching, 2);

    Matching2 loweRatioMatching;
    for (auto& match : initialMatching) {
        if (SFMUtilities::PassesLoweRatioTest(match)) {
            loweRatioMatching.push_back(match[0]);
        }
    }

    // TODO: geometric verification of matches by fundamental matrix

    return loweRatioMatching;
}
