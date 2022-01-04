//
// Created by Megan Finch on 28/12/2021.
//

#include <headers/sfm_util.h>
#include <opencv2/calib3d.hpp>
#include "headers/constants.h"
#include "headers/exceptions.h"

bool SFMUtilities::PassesLoweRatioTest(const std::vector<cv::DMatch> &match) {
    return match.size() == 2 && static_cast<double>(match[0].distance) < static_cast<double>(match[1].distance) * LOWE_RATIO;
}

Pose
SFMUtilities::recoverPoseFromMatches(Camera &cam1, Camera &cam2,
                                     Features &features1, Features &features2,
                                     Matching2 &matching,
                                     Matching2 &prunedMatching)
{
    // Get two arrays of matching points
    std::vector<int> backRef1;
    std::vector<int> backRef2;
    std::vector<cv::Point2d> points1;
    std::vector<cv::Point2d> points2;
    SFMUtilities::getAlignedPointsFromMatch(features1, features2, matching, points1, points2, backRef1, backRef2);

    // TODO: for now, assumes cam1.K == cam2.K
    // but this is not always true e.g. if images are different size
    double focal = cam1.getFocalLength();
    auto pp = cam1.getCentre();

    if (points1.size() < POINTS_NEEDED_ESSENTIAL_MATRIX) {
        throw std::runtime_error("Not enough points to compute essential matrix.");
    }

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

Pose SFMUtilities::recoverPoseFrom2D3DMatches(Camera& camera, Image2D3DMatch matching) {
    cv::Matx31d rvec, tvec;
    cv::Mat inliers;

    cv::solvePnPRansac(matching.points3D,
                       matching.points2D,
                       camera.getCameraMatrix(),
                       camera.getDistortion(),
                       rvec,
                       tvec,
                       false,
                       100,
                       (float)RANSAC_THRESHOLD,
                       0.99,
                       inliers
                       );

    // Check inliers ratio, reject if too small
    double inliersRatio = ((double)cv::countNonZero(inliers)) / ((double)matching.points2D.size());
    if (inliersRatio < POSE_INLIERS_MINIMAL_RATIO) {
        std::string msg = "Pose inliers ratio is too small: ";
        msg.append(std::to_string(cv::countNonZero(inliers)));
        msg.append(" / ");
        msg.append(std::to_string(matching.points2D.size()));

        throw std::runtime_error(msg);
    }

    Pose pose = Pose(rvec, tvec);

    return pose;
}

PointCloud SFMUtilities::triangulateViews(ImageID img1, ImageID img2,
                                   Camera& cam1, Camera& cam2,
                                   Features& features1, Features& features2,
                                   Matching2& matching,
                                   Pose& pose1, Pose& pose2)
{
    // Get two arrays of matching points
    std::vector<cv::Point2d> alignedPoints1;
    std::vector<cv::Point2d> alignedPoints2;
    std::vector<int> backReference1;
    std::vector<int> backReference2;

    SFMUtilities::getAlignedPointsFromMatch(features1, features2,
                                            matching,
                                            alignedPoints1, alignedPoints2,
                                            backReference1, backReference2);

    cv::Mat normalisedPoints1;
    cv::Mat normalisedPoints2;
    cv::undistortPoints(alignedPoints1, normalisedPoints1, cam1.getCameraMatrix(), cv::Mat());
    cv::undistortPoints(alignedPoints2, normalisedPoints2, cam2.getCameraMatrix(), cv::Mat());

    cv::Mat points3dHomogenous;
    cv::triangulatePoints(pose1.getProjectionMatrix(), pose2.getProjectionMatrix(), normalisedPoints1, normalisedPoints2, points3dHomogenous);

    cv::Mat points3d;
    cv::convertPointsFromHomogeneous(points3dHomogenous.t(), points3d);

    // TODO: reprojection errors
    auto reprojectionErrors1 = SFMUtilities::getReprojectionErrors(alignedPoints1, points3d, cam1, pose1);
    auto reprojectionErrors2 = SFMUtilities::getReprojectionErrors(alignedPoints2, points3d, cam2, pose2);

    // Populate PointCloud
    PointCloud pc;

    for (size_t i = 0; i < points3d.rows; i++) {
        if (reprojectionErrors1[i] > REPROJECTION_ERROR_THRESHOLD or
            reprojectionErrors2[i] > REPROJECTION_ERROR_THRESHOLD) {
            continue;
        }

        Point3DInMap p;
        p.pt = cv::Point3d(points3d.at<double>(i, 0),
                points3d.at<double>(i,1),
                points3d.at<double>(i,2));

        p.originatingViews[img1] = backReference1[i];
        p.originatingViews[img2] = backReference2[i];

        pc.addPoint(p);
    }

    return pc;
}

void SFMUtilities::getAlignedPointsFromMatch(Features& queryFeatures, Features& trainFeatures,
                                             Matching2& matching,
                                             std::vector<cv::Point2d>& queryAlignedPoints, std::vector<cv::Point2d>& trainAlignedPoints,
                                             std::vector<int>& queryBackReference, std::vector<int>& trainBackReference)
{
    queryAlignedPoints.clear();
    trainAlignedPoints.clear();
    queryBackReference.clear();
    trainBackReference.clear();

    for (auto match : matching) {
        queryAlignedPoints.push_back(queryFeatures.getPoint(match.queryIdx));
        trainAlignedPoints.push_back(trainFeatures.getPoint(match.trainIdx));
        queryBackReference.push_back(match.queryIdx);
        trainBackReference.push_back(match.trainIdx);
    }
}

std::vector<double>
SFMUtilities::getReprojectionErrors(const std::vector<cv::Point2d>& points2d, const cv::Mat& points3d, const Camera &camera, const Pose &pose) {
    cv::Matx31d rvec = pose.getRotationVector();
    auto tvec = pose.getTranslationVector();

    // Reproject points
    std::vector<cv::Point2d> reprojectedPoints(points2d.size());
    cv::projectPoints(points3d, rvec, tvec, camera.getCameraMatrix(), cv::Mat(), reprojectedPoints);

    // Calculate errors
    std::vector<double> reprojectionErrors(points2d.size());
    for (size_t i = 0; i < points3d.rows; i++) {

        double error = cv::norm(reprojectedPoints[i] - points2d[i]);
        reprojectionErrors[i] = error;
    }

    if (DEFAULT_DEBUG >= DebugLevel::DETAILED) {
        for (size_t i = 0; i < points3d.rows; i++) {
            std::cout << "-----Point #" << i << "-----" << std::endl;
            std::cout << "Actual: " << points2d[i] << std::endl;
            std::cout << "Reprojected: " << reprojectedPoints[i] << std::endl;
            std::cout << "Error: " << reprojectionErrors[i] << std::endl;
        }
    }

    return reprojectionErrors;
}

Image2D3DMatch
SFMUtilities::find2D3DMatches(ImageID imageId,
                              Features &imageFeatures,
                              Matches &matches,
                              PointCloud &pointCloud)
{
    Image2D3DMatch match2D3D;

    for (const Point3DInMap& cloudPoint : pointCloud) {
        bool found2DPoint = false;

        for (const auto& origViewAndPoint : cloudPoint.originatingViews) {
            const ImageID originatingViewIndex = origViewAndPoint.first;
            const int originatingPointIndex = origViewAndPoint.second;

            for (const cv::DMatch& m : matches.getMatchingBetween(imageId, originatingViewIndex)) {
                int matched2DPointInNewView = -1;

                if (originatingViewIndex < imageId) {
                    if (m.queryIdx == originatingPointIndex) {
                        matched2DPointInNewView = m.trainIdx;
                    }
                } else {
                    if (m.trainIdx == originatingPointIndex) {
                        matched2DPointInNewView = m.queryIdx;
                    }
                }

                if (matched2DPointInNewView > -1) {
                    match2D3D.points2D.push_back(imageFeatures.getPoint(matched2DPointInNewView));
                    match2D3D.points3D.push_back(cloudPoint.pt);
                    found2DPoint = true;
                    break; // out of matches loop
                }
            }

            if (found2DPoint) { break; } // out of originating views loop
        }
    }

    return match2D3D;
}
