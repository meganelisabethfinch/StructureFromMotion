//
// Created by Megan Finch on 13/02/2022.
//

#ifndef SFM_LINEAR_TRIANGULATION_H
#define SFM_LINEAR_TRIANGULATION_H

#include <headers/sfm_util.h>
#include <opencv2/calib3d.hpp>
#include "triangulator.h"

class LinearTriangulator : public Triangulator {
public:
    PointCloud triangulateImages(ImageID img1, ImageID img2,
                                 Camera &cam1, Camera &cam2,
                                 Features &features1, Features &features2,
                                 Matching2 &matching,
                                 Pose &pose1, Pose &pose2) override {
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
        cv::triangulatePoints(pose1.getProjectionMatrix(), pose2.getProjectionMatrix(), normalisedPoints1,
                              normalisedPoints2, points3dHomogenous);

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
                               points3d.at<double>(i, 1),
                               points3d.at<double>(i, 2));

            p.originatingViews[img1] = backReference1[i];
            p.originatingViews[img2] = backReference2[i];

            pc.addPoint(p);
        }

        return pc;
    }
};

#endif //SFM_LINEAR_TRIANGULATION_H
