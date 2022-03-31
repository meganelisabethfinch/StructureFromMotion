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
enum class TriangulatorType { LINEAR, MIDPOINT };
enum class BundleAdjusterType { OFF, BASIC, ZHANG };
enum class FilterType { STATISTICAL, RADIAL };
enum class OutputType { PLY_POINT_CLOUD, PLY_CAMERAS, PCD_POINT_CLOUD, VTK_MESH };

enum DebugLevel { SIMPLE, VERBOSE };
enum VisualDebugLevel { NONE, SHOW_EXAMPLES };

// Matches between two images
typedef std::vector<cv::DMatch> Matching2;
typedef std::vector<std::vector<Matching2>> MatchMatrix;

// 1x3 rotation in angle-axis form, followed by 1x3 translation
typedef cv::Matx<double, 1, 6> PoseVector;


struct Image2D3DMatch {
    std::vector<cv::Point2d> points2D;
    std::vector<cv::Point3d> points3D;

    [[nodiscard]] size_t size() const {
        return points2D.size();
    }
};


#endif //SFM_TYPES_H
