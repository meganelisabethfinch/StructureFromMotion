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

    ImageID baseline1;
    ImageID baseline2;

    DetectorType detectorType;
    MatcherType matcherType;
};

// Matches between two images
typedef std::vector<cv::DMatch> Matching2;
typedef std::vector<std::vector<Matching2>> MatchMatrix;

// 1x3 rotation in angle-axis form, followed by 1x3 translation
typedef cv::Matx<double, 1, 6> PoseVector;

struct Image2D3DMatch {
    std::vector<cv::Point2d> points2D;
    std::vector<cv::Point3d> points3D;
};

#endif //SFM_TYPES_H
