//
// Created by Megan Finch on 28/12/2021.
//

#include <headers/sfm_util.h>
#include <opencv2/calib3d.hpp>
#include "headers/constants.h"

bool SFMUtilities::PassesLoweRatioTest(const std::vector<cv::DMatch> &match) {
    return match.size() == 2 && static_cast<double>(match[0].distance) < static_cast<double>(match[1].distance) * LOWE_RATIO;
}

Pose
SFMUtilities::recoverPose(Camera &cam1, Camera &cam2,
                          Features &features1, Features &features2,
                          Matching2 &matching,
                          Matching2 &prunedMatching)
{
    // Get two arrays of matching points
    auto points1 = features1.GetPointsFromMatches(matching, true);
    auto points2 = features2.GetPointsFromMatches(matching, false);

    // TODO: for now, assumes cam1.K == cam2.K
    // but this is not always true
    double focal = cam1.getFocalLength();
    auto pp = cam1.getCentre();

    // Find essential matrix
    cv::Mat E, R, t;
    cv::Mat mask;
    E = cv::findEssentialMat(points1, points2, focal, pp, cv::RANSAC, 0.999, 1.0, mask);

    cv::recoverPose(E, points1, points2, R, t, focal, pp, mask);

    // Set pose1 and pose2
    // auto P1 = cv::Matx34d::eye();
    auto pose2 = Pose(cv::Matx34d(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0),
                        R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1),
                        R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2)));

    // Populate pruned matches
    prunedMatching.clear();
    for (size_t i = 0; i < mask.rows; i++) {
        if (mask.at<uchar>(i)) {
            prunedMatching.push_back(matching[i]);
        }
    }

    std::cout << "Pruned matching: " << prunedMatching.size() << " of " << matching.size() << " matches kept." << std::endl;

    return pose2;
}



PointCloud SFMUtilities::triangulateViews(ImageID img1, ImageID img2,
                                   Camera& cam1, Camera& cam2,
                                   Features& features1, Features& features2,
                                   Matching2& matching,
                                   Pose& pose1, Pose& pose2)
{
    // Get two arrays of matching points
    auto points1 = features1.GetPointsFromMatches(matching, true);
    auto points2 = features2.GetPointsFromMatches(matching, false);

    cv::Mat normalisedPoints1;
    cv::Mat normalisedPoints2;
    cv::undistortPoints(points1, normalisedPoints1, cam1.getCameraMatrix(), cv::Mat());
    cv::undistortPoints(points2, normalisedPoints2, cam2.getCameraMatrix(), cv::Mat());

    cv::Mat points3dHomogenous;
    cv::triangulatePoints(pose1.getProjectionMatrix(), pose2.getProjectionMatrix(), normalisedPoints1, normalisedPoints2, points3dHomogenous);

    cv::Mat points3d;
    cv::convertPointsFromHomogeneous(points3dHomogenous.t(), points3d);

    // TODO: reprojection errors


    // Populate PointCloud
    PointCloud pc;

    for (size_t i = 0; i < points3d.rows; i++) {
        Point3DInMap p;
        p.pt = cv::Point3d(points3d.at<float>(i, 0), points3d.at<float>(i,1), points3d.at<float>(i,2));

        // p.originatingViews[img1] =

        pc.addPoint(p);
    }

    return pc;
}