//
// Created by Megan Finch on 28/12/2021.
//

#include <headers/features.h>
#include <headers/image.h>
#include <headers/sfm_util.h>
#include <headers/constants.h>
#include <iostream>
#include <opencv2/calib3d.hpp>

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
    std::vector<cv::Point2d> source, destination;

    for (auto& match : initialMatching) {
        if (SFMUtilities::PassesLoweRatioTest(match)) {
            loweRatioMatching.push_back(match[0]);

            // Track coordinates of keypoints involved in good matches
            // this = i, other = j, i < j. Smaller image id corresponds to queryIdx.
            source.push_back(_points2d[match[0].queryIdx]);
            destination.push_back(other.getPoint(match[0].trainIdx));
        }
    }

    Matching2 verifiedMatching;
    // Geometric verification by fundamental matrix
    if (loweRatioMatching.size() > POINTS_NEEDED_FUNDAMENTAL_MATRIX) {
        std::vector<uchar> mask;
        cv::Mat F = findFundamentalMat(source, destination, cv::FM_RANSAC, 3.0, 0.99, mask);

        // Store fundamental matrix - should one be F.inverse?
        // images[i].setFundamentalMatrix(images[j].getId(), F);
        // images[j].setFundamentalMatrix(images[i].getId(), F);

        for (int k = 0; k < mask.size(); k++) {
            if (mask[k]) {
                // Classify this as a good match
                verifiedMatching.push_back(loweRatioMatching[k]);
            }
        }
    }

    if (DEFAULT_DEBUG >= DebugLevel::VERBOSE) {
        std::cout << "Initial KNN Matching: " << initialMatching.size() << " matches" << std::endl;
        std::cout << "Lowe Ratio Matching: " << loweRatioMatching.size() << " matches" << std::endl;
        std::cout << "Geometrically verified Matching: " << verifiedMatching.size() << " matches" << std::endl;
    }

    // out = loweRatioMatching;
    out = verifiedMatching;
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
    // TODO: add check that index is in size
    return _points2d[i];
}
