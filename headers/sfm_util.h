//
// Created by Megan Finch on 28/12/2021.
//

#ifndef SFM_UTIL_H
#define SFM_UTIL_H

#include <vector>
#include <opencv2/core/types.hpp>
#include "camera.h"
#include "features.h"
#include "pose.h"
#include "point_cloud.h"

class SFMUtilities {
public:
    static bool PassesLoweRatioTest(const std::vector<cv::DMatch>& match);

    static Pose recoverPose(Camera &cam1, Camera &cam2,
                            Features &features1, Features &features2,
                            Matching2 &matching,
                            Matching2 &prunedMatching);

    static PointCloud triangulateViews(ImageID img1, ImageID img2,
            Camera& cam1, Camera& cam2,
            Features& features1, Features& features2,
            Matching2& matching,
            Pose& pose1, Pose& pose2);

    static void getAlignedPointsFromMatch(Features& queryFeatures, Features& trainFeatures,
                                          Matching2& matching,
                                          std::vector<cv::Point2d>& queryAlignedPoints, std::vector<cv::Point2d>& trainAlignedPoints,
                                          std::vector<int>& queryBackReference, std::vector<int>& trainBackReference);

    static std::vector<double> getReprojectionErrors(const std::vector<cv::Point2d>& points2d,
                                                     const cv::Mat& points3d,
                                                     const Camera& camera,
                                                     const Pose& pose);
};

#endif //SFM_UTIL_H
