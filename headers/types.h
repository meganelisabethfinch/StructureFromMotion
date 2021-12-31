//
// Created by Megan Finch on 23/12/2021.
//

#ifndef SFM_TYPES_H
#define SFM_TYPES_H

#include <cstddef>
#include <string>
#include <opencv2/features2d.hpp>

typedef size_t ImageID;

enum class DetectorType { SIFT, ORB };
typedef cv::DescriptorMatcher::MatcherType MatcherType;
enum DebugLevel { BASIC, DETAILED, EXAMPLES };

struct Args {
    std::string inputImageDir;
    std::string outputDir;
    // TODO: add manually specified baseline pair
    DetectorType detectorType;
    MatcherType matcherType;
};

// Matches between two images
typedef std::vector<cv::DMatch> Matching2;
typedef std::vector<std::vector<Matching2>> MatchMatrix;

#endif //SFM_TYPES_H
