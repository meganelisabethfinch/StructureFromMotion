//
// Created by Megan Finch on 13/02/2022.
//

#ifndef SFM_MIDPOINT_TRIANGULATION_H
#define SFM_MIDPOINT_TRIANGULATION_H

#include "triangulator.h"

class MidpointTriangulator : public Triangulator {
public:
    /*
     * Finds the approximate intersection of two lines, represented as
     * L1 = { p1 = q1 + lambda1 * v1 }
     * and
     * L2 = { p2 = q2 + lambda2 * v2 }
     *
     * @param q1 - a point on L1
     * @param q2 - a point on L2
     * @param v1 - the direction of L1
     * @param v2 - the direction of L2
     */
     static cv::Point3d triangulatePoint(const cv::Vec3d& q1, const cv::Vec3d& q2,
                                 const cv::Vec3d& v1, const cv::Vec3d& v2) {
        cv::Vec3d vcross = v1.cross(v2);
        double denom = 1.0 / vcross.dot(vcross);

        double lambda1 = ((q2 - q1).cross(v2)).dot(vcross) * denom;
        double lambda2 = ((q2 - q1).cross(v1)).dot(vcross) * denom;

        cv::Vec3d p1 = q1 + lambda1 * v1;
        cv::Vec3d p2 = q2 + lambda2 * v2;
        cv::Vec3d midpoint = (p1 + p2) * 0.5;

        return { midpoint };
    }

    static std::pair<cv::Vec3d, cv::Vec3d> backprojectPointToRay(Camera& camera,
                                                                 Pose& pose,
                                                                 cv::Point2d& imagePoint) {
        cv::Matx33d M = camera.getCameraMatrix() * pose.getRotationMatrix();
        cv::Matx31d t = camera.getCameraMatrix() * pose.getTranslationVector();

        cv::Matx31d res = - M.inv() * t;
        cv::Vec3d q = { res(0), res(1), res(2) };

        // TODO: questionable?! What's going on with homogenous coords?
        cv::Vec3d homogenousPoint = { imagePoint.x, imagePoint.y, 1 };
        cv::Vec3d v = M.inv() * homogenousPoint;

        return { q, v };
    }

    PointCloud triangulateImages(ImageID img1, ImageID img2,
                                 Camera& cam1, Camera& cam2,
                                 Features& features1, Features& features2,
                                 Matching2& matching,
                                 Pose& pose1, Pose& pose2) override
    {
        PointCloud pc;

        // Implements equation 6.14 from MVG in Computer Vision (Hartley & Zisserman)
        cv::Matx33d M1 = cam1.getCameraMatrix() * pose1.getRotationMatrix();
        cv::Matx33d M2 = cam2.getCameraMatrix() * pose2.getRotationMatrix();
        cv::Matx31d t1 = cam1.getCameraMatrix() * pose1.getTranslationVector();
        cv::Matx31d t2 = cam1.getCameraMatrix() * pose2.getTranslationVector();

        for (auto& match : matching) {
            // Get image points
            cv::Point2d pt1 = features1.getPoint(match.trainIdx);
            cv::Point2d pt2 = features2.getPoint(match.queryIdx);

            // Backprojection
            auto L1 = backprojectPointToRay(cam1, pose1, pt1);
            auto L2 = backprojectPointToRay(cam2, pose2, pt2);

            // Do the triangulation
            auto pt = triangulatePoint(L1.first, L2.first, L1.second, L2.second);

            // Add to point cloud
            Point3DInMap pt3d;
            pt3d.pt = pt;
            pt3d.originatingViews.insert({img1, match.trainIdx});
            pt3d.originatingViews.insert({img2, match.queryIdx});
            pc.addPoint(pt3d);
        }
        return pc;
    }
};

#endif //SFM_MIDPOINT_TRIANGULATION_H
