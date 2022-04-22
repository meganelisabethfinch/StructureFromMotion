//
// Created by Megan Finch on 28/12/2021.
//

#include <headers/sfm_util.h>
#include <opencv2/calib3d.hpp>
#include <ceres/ceres.h>
#include <ceres/solver.h>
#include <headers/cost/simple_reprojection_error.h>
#include <ceres/loss_function.h>
#include "headers/constants.h"
#include "headers/image_pair.h"

bool SFMUtilities::PassesLoweRatioTest(const std::vector<cv::DMatch> &match) {
    return match.size() == 2 && static_cast<double>(match[0].distance) < static_cast<double>(match[1].distance) * LOWE_RATIO;
}

Pose
SFMUtilities::recoverPoseFromMatches(Camera &cam1, Camera &cam2,
                                     Features &features1, Features &features2,
                                     Matching2 &matching,
                                     cv::Mat& mask)
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
        std::string err = "Not enough points to compute essential matrix: ";
        err.append(std::to_string(points1.size()));
        err.append(" / ");
        err.append(std::to_string(POINTS_NEEDED_ESSENTIAL_MATRIX));
        throw std::runtime_error(err);
    }

    // Find essential matrix
    cv::Mat E, R, t;
    E = cv::findEssentialMat(points1, points2, focal, pp, cv::RANSAC, 0.999, 1.0, mask);

    cv::recoverPose(E, points1, points2, R, t, focal, pp, mask);

    // Set pose1 and pose2
    // auto P1 = cv::Matx34d::eye();
    auto pose2 = Pose(cv::Matx34d(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0),
                        R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1),
                        R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2)));

    return pose2;
}

Pose SFMUtilities::recoverPoseFrom2D3DMatches(Camera& camera, Image2D3DMatch matching) {
    cv::Matx31d rvec, tvec;
    cv::Mat inliers;

    try {
        cv::solvePnPRansac(matching.points3D,
                           matching.points2D,
                           camera.getCameraMatrix(),
                           camera.getDistortion(),
                           rvec,
                           tvec,
                           false,
                           PNP_RANSAC_MAX_ITERATIONS,
                           (float) RANSAC_THRESHOLD,
                           PNP_RANSAC_CONFIDENCE,
                           inliers
        );
    } catch (cv::Exception& e) {
        throw std::runtime_error(e.msg);
    }

    // Check inliers ratio, reject if too small
    double inliersRatio = ((double)cv::countNonZero(inliers)) / ((double)matching.points2D.size());
    if (inliersRatio < POSE_INLIERS_MINIMAL_RATIO) {
        std::string msg = "Pose inliers ratio is too small: ";
        msg.append(std::to_string(cv::countNonZero(inliers)));
        msg.append(" / ");
        msg.append(std::to_string(matching.points2D.size()));

        throw std::runtime_error(msg);
    }

    return { rvec, tvec };
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
    auto tvec = pose.getTranslationVectorAlt();

    // Reproject points
    std::vector<cv::Point2d> reprojectedPoints(points2d.size());
    cv::projectPoints(points3d, rvec, tvec, camera.getCameraMatrix(), cv::Mat(), reprojectedPoints);

    // Calculate errors
    std::vector<double> reprojectionErrors(points2d.size());
    for (size_t i = 0; i < points3d.rows; i++) {

        double error = cv::norm(reprojectedPoints[i] - points2d[i]);
        reprojectionErrors[i] = error;
    }

    if (DEFAULT_DEBUG >= DebugLevel::VERBOSE) {
        for (size_t i = 0; i < points3d.rows; i++) {
            std::cout << "-----Point #" << i << "-----" << std::endl;
            std::cout << "Actual: " << points2d[i] << std::endl;
            std::cout << "Reprojected: " << reprojectedPoints[i] << std::endl;
            std::cout << "Error: " << reprojectionErrors[i] << std::endl;
        }
    }

    return reprojectionErrors;
}

double SFMUtilities::globalReprojectionError(PointCloud& pointCloud,
                                      const std::set<ImageID>& registeredImages,
                                      std::map<ImageID, Pose>& cameraPoses,
                                      std::vector<Camera>& cameras,
                                      std::vector<Features>& features,
                                      LossType lossType = LossType::NULL_LOSS)
{
    ceres::Problem problem;

    std::map<ImageID, PoseVector> cameraPoses6d;
    for (auto i : registeredImages) {
        auto pv = cameraPoses.at(i).toPoseVector();
        cameraPoses6d[i] = pv;
    }

    double focal = cameras.at(0).getFocalLength();

    std::vector<cv::Vec3d> points3d(pointCloud.size());

    for (size_t i = 0; i < pointCloud.size(); i++) {
        const Point3DInMap& p = pointCloud[i];
        points3d[i] = cv::Vec3d(p.pt.x, p.pt.y, p.pt.z);

        for (const auto& kv : p.originatingViews) {
            cv::Point2d p2d = features.at(kv.first).getPoint(kv.second);

            // Subtract centre of projection, since the optimiser doesn't know what it is
            p2d.x -= cameras.at(kv.first).getCentre().x;
            p2d.y -= cameras.at(kv.first).getCentre().y;

            ceres::CostFunction* cost_function = SimpleReprojectionError::Create(p2d.x, p2d.y);
            ceres::LossFunction* loss_function;

            switch (lossType) {
                case LossType::NULL_LOSS:
                    loss_function = nullptr;
                    break;
                case LossType::HUBER:
                    loss_function = new ceres::HuberLoss(1.0);
                    break;
                case LossType::SOFTLONE:
                    loss_function = new ceres::SoftLOneLoss(1.0);
                    break;
                case LossType::CAUCHY:
                    loss_function = new ceres::CauchyLoss(1.0);
                    break;
            }
            PoseVector& pose = cameraPoses6d[kv.first];

            problem.AddResidualBlock(cost_function,
                                     loss_function,
                                     pose.val,
                                     points3d[i].val,
                                     &focal);
        }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 0;
    options.eta = 1e-2;
    options.max_solver_time_in_seconds = 10;
    options.logging_type = ceres::LoggingType::SILENT;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    return summary.initial_cost;
}

Image2D3DMatch
SFMUtilities::find2D3DMatches(ImageID imageId,
                              Features &imageFeatures,
                              Matches &matches,
                              PointCloud &pointCloud)
{
    Image2D3DMatch match2D3D;

    // Scan all 3D points in point cloud
    for (const Point3DInMap& cloudPoint : pointCloud) {
        bool found2DPoint = false;

        for (const auto& origViewAndPoint : cloudPoint.originatingViews) {
            const ImageID originatingViewIndex = origViewAndPoint.first;
            const int originatingPointIndex = origViewAndPoint.second;

            auto ip = ImagePair(imageId, originatingViewIndex);
            auto list_matches = matches.get(ip);
            for (const cv::DMatch& m : list_matches) {
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

cv::Matx33d SFMUtilities::pruneMatchesByFundamentalMatrix(const std::vector<cv::Point2d>& source,
                                                          const std::vector<cv::Point2d>& destination,
                                                          const Matching2& matching,
                                                          Matching2& prunedMatching) {

    prunedMatching.clear();
    if (matching.size() > POINTS_NEEDED_FUNDAMENTAL_MATRIX) {
        std::vector<uchar> mask;
        cv::Mat F = cv::findFundamentalMat(source,
                                       destination,
                                       cv::FM_RANSAC,
                                       FM_RANSAC_THRESHOLD,
                                       FM_RANSAC_CONFIDENCE,
                                       mask);

        cv::Matx33d result;
        if (F.rows == 3 && F.cols == 3) {
            result = cv::Matx33d(F);
        } else {
            // cv::findFundamentalMat sometimes returns a 9x3 with 3 possible Fs
            // We'll just ignore/reject this case.
            throw std::runtime_error("Could not compute fundamental matrix with enough certainty.");
        }

        for (size_t i = 0; i < mask.size(); i++) {
            if (mask[i]) {
                // Classify this as a good match
                prunedMatching.push_back(matching[i]);
            }
        }

        return result;
    } else {
        throw std::runtime_error("Not enough points to compute fundamental matrix.");
    }
}

std::map<double, ImagePair>
SFMUtilities::SortViewsForBaseline(std::vector<Features> &mImageFeatures, Matches &mFeatureMatchMatrix) {
    // Sort pairs by ascending homography inliers ratio.
    // Pairs with too few points are last.

    std::map<double, ImagePair> sortedPairs;
    const size_t numImages = mImageFeatures.size();
    for (size_t i = 0; i < numImages - 1; i++) {
        for (size_t j = i + 1; j < numImages; j++) {
            auto ip = ImagePair(i,j);
            auto& matching2 = mFeatureMatchMatrix.get(ip);
            if (matching2.size() < POINTS_NEEDED_HOMOGRAPHY_MATRIX) {
                sortedPairs.emplace(1.0, ip);
                continue;
            }

            // Find ratio of homography inliers
            const int numInliers = SFMUtilities::CountHomographyInliers(mImageFeatures[i], mImageFeatures[j], matching2);

            const double inliersRatio = ((double) numInliers) / ((double)matching2.size());
            sortedPairs.emplace(inliersRatio, ImagePair( i, j ));
        }
    }

    return sortedPairs;
}

int SFMUtilities::CountHomographyInliers(Features &left, Features &right, Matching2 &matches) {
    std::vector<cv::Point2d> alignedLeft;
    std::vector<cv::Point2d> alignedRight;
    std::vector<int> leftBackReference;
    std::vector<int> rightBackReference;
    SFMUtilities::getAlignedPointsFromMatch(left, right, matches, alignedLeft, alignedRight, leftBackReference, rightBackReference);

    cv::Mat inlierMask;
    cv::Mat H;
    if (matches.size() >= 4) {
        H = cv::findHomography(alignedLeft, alignedRight, cv::RANSAC, RANSAC_THRESHOLD, inlierMask);
    }

    if (matches.size() < 4 || H.empty()) {
        return 0;
    }

    return cv::countNonZero(inlierMask);
}
