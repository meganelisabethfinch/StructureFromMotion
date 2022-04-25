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
#include "matches.h"
#include "image_pair.h"

class SFMUtilities {
public:
    static bool PassesLoweRatioTest(const std::vector<cv::DMatch>& match);

    static Pose recoverPoseFromMatches(Camera &cam1, Camera &cam2,
                                       Features &features1, Features &features2,
                                       Matching2 &matching,
                                       cv::Mat& mask);

    static Pose recoverPoseFrom2D3DMatches(Camera& camera,
                                           Image2D3DMatch matching);

    static void getAlignedPointsFromMatch(Features& queryFeatures, Features& trainFeatures,
                                          Matching2& matching,
                                          std::vector<cv::Point2d>& queryAlignedPoints, std::vector<cv::Point2d>& trainAlignedPoints,
                                          std::vector<int>& queryBackReference, std::vector<int>& trainBackReference);

    static std::vector<double> getReprojectionErrors(const std::vector<cv::Point2d>& points2d,
                                                     const cv::Mat& points3d,
                                                     const Camera& camera,
                                                     const Pose& pose);

    /*
     * Quick and hacky way to count radial outliers for evaluation.
     * Copy of code from ROR filter, without updating the point cloud.
     */
    static size_t countRadialOutliers(PointCloud& pc, double radiusSearch, int minNeighborsInRadius);

    /*
     * Quick and hacky way to count statistical outliers for evaluation.
     * Copy of code from SOR filter, without updating the point cloud.
     */
    static size_t countStatisticalOutliers(PointCloud& pc, int k, double stddev_mult);

    static Image2D3DMatch find2D3DMatches(ImageID imageId,
                                          Features& imageFeatures,
                                          Matches& matches,
                                          PointCloud& pointCloud);

    static cv::Matx33d pruneMatchesByFundamentalMatrix(const std::vector<cv::Point2d>& source,
                                                       const std::vector<cv::Point2d>& destination,
                                                       const Matching2& matching,
                                                       Matching2& prunedMatching);

    static std::map<double, ImagePair> SortViewsForBaseline(std::vector<Features>& mImageFeatures,
                                                            Matches& mFeatureMatchMatrix
                                                            );

    static int CountHomographyInliers(Features& left, Features& right, Matching2& matches);
};

#endif //SFM_UTIL_H
