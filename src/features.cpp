//
// Created by Megan Finch on 28/12/2021.
//

#include <headers/features.h>
#include <headers/image.h>
#include <headers/sfm_util.h>
#include <headers/constants.h>
#include <iostream>

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

// TODO: move to Matches class?
void Features::findMatchesWith(const cv::Ptr<cv::DescriptorMatcher> &matcher, Features &other, Matching2& out) {
    std::vector<std::vector<cv::DMatch>> initialMatching;
    matcher->knnMatch(_descriptors, other.getCVDescriptors(), initialMatching, 2);

    Matching2 loweRatioMatching;
    for (auto& match : initialMatching) {
        if (SFMUtilities::PassesLoweRatioTest(match)) {
            loweRatioMatching.push_back(match[0]);
        }
    }

    // TODO: geometric verification of matches by fundamental matrix

    if (DEFAULT_DEBUG >= DebugLevel::DETAILED) {
        std::cout << "Initial KNN Matching: " << initialMatching.size() << " matches" << std::endl;
        std::cout << "Lowe Ratio Matching: " << loweRatioMatching.size() << " matches" << std::endl;
    }

    out = loweRatioMatching;
}

std::vector<cv::Point2d> Features::GetPointsFromMatches(Matching2 &matching, bool query, std::vector<int>& backReference) {
    std::vector<cv::Point2d> points;
    // TODO: we also want to keep the query or trainIdx
    if (query) {
        for (auto match : matching) {
            points.push_back(_points2d[match.queryIdx]);
        }
    } else {
        for (auto match : matching) {
            points.push_back(_points2d[match.trainIdx]);
        }
    }
    return points;
}

size_t Features::size() const {
    return _keypoints.size();
}

cv::Point2d Features::getPoint(size_t i) const {
    return _points2d[i];
}
