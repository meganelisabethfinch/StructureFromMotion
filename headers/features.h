//
// Created by Megan Finch on 28/12/2021.
//

#ifndef SFM_FEATURES_H
#define SFM_FEATURES_H

#include <opencv2/core/cvstd_wrapper.hpp>
#include "image.h"

class Features {
private:
    std::vector<cv::KeyPoint> _keypoints;
    std::vector<cv::Point2d> _points2d;
    cv::Mat _descriptors;

public:
    Features(const cv::Ptr<cv::FeatureDetector>& detector, const Image& image);

    void findMatchesWith(const cv::Ptr<cv::DescriptorMatcher>& matcher, Features& other, Matching2& out);

    std::vector<cv::KeyPoint>& getCVKeyPoints();

    cv::Mat& getCVDescriptors();

    cv::Point2d getPoint(size_t i);

    std::vector<cv::Point2d> GetPointsFromMatches(Matching2& matching, bool query, std::vector<int>& backReference);

    [[nodiscard]] size_t size() const;
};

#endif //SFM_FEATURES_H
