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

    /*
     * Finds the 3D point p on the virtual image plane of a camera with centre C,
     * corresponding to a 2D image point u.
     *
     * @param u - a 2D image point
     * @param K - the intrinsic camera matrix
     * @param P - the camera pose, where P = [R|t]
     */
    static cv::Point3d backProjectPointToPoint(const cv::Point2d& u,
                                               const Camera& K,
                                               const Pose& P) {
        // 1.
        // cv::Vec3d u_prime = { u.x, u.y, K.getFocalLength() };
        // auto p = K.getCameraMatrix() * P.getRotationMatrix() * u_prime + K.getCameraMatrix() * P.getTranslationVector();

        // 3.
        // cv::Vec3d u_prime = { u.x, u.y, 1 };
        // auto p = P.getRotationMatrix() * K.getCameraMatrix().inv() * u_prime + P.getTranslationVector();

        // 4.
        // cv::Vec3d u_prime = { u.x, u.y, 1 };
        // auto p = K.getCameraMatrix() * P.getRotationMatrix() * K.getCameraMatrix().inv() * u_prime + K.getCameraMatrix() * P.getTranslationVector();

        // 5.
        // cv::Vec3d u_prime = { u.x, u.y, K.getFocalLength() };
        // auto p = P.getRotationMatrix() * K.getCameraMatrix().inv() * u_prime + P.getTranslationVector();

        // 8.
        // cv::Vec3d u_prime = { u.x, u.y, K.getFocalLength() };
        // auto p = P.getRotationMatrix().inv() * K.getCameraMatrix().inv() * u_prime - P.getTranslationVector();

        // 9.
        cv::Vec3d u_prime = { u.x, u.y, K.getFocalLength() };
        auto p = P.getRotationMatrix() * u_prime + P.getTranslationVector();
        return { p(0), p(1), p(2) };
    }

    static std::pair<cv::Vec3d, cv::Vec3d> backProjectPointToRay(const cv::Point2d& u,
                                                                 const Camera& K,
                                                                 const Pose& P) {
        auto p_pt = backProjectPointToPoint(u,K,P);
        auto C_mat = P.getTranslationVector();

        // Reformat as vectors
        cv::Vec3d p = {p_pt.x, p_pt.y, p_pt.z};
        cv::Vec3d C = { C_mat(0), C_mat(1), C_mat(2) };
        return { C, C - p };
    }

    PointCloud triangulateImages(ImageID img1, ImageID img2,
                                 Camera& cam1, Camera& cam2,
                                 Features& features1, Features& features2,
                                 Matching2& matching,
                                 Pose& pose1, Pose& pose2) override
    {
        PointCloud pc;

        for (auto& match : matching) {
            // Get image points
            // Note: smaller image id always corresponds to queryIdx
            cv::Point2d pt1 = features1.getPoint(match.queryIdx);
            cv::Point2d pt2 = features2.getPoint(match.trainIdx);

            // Backprojection
            auto L1 = backProjectPointToRay(pt1, cam1, pose1);
            auto L2 = backProjectPointToRay(pt2, cam2, pose2);

            // Do the triangulation
            auto pt = triangulatePoint(L1.first, L2.first, L1.second, L2.second);

            // Add to point cloud
            Point3DInMap pt3d;
            pt3d.pt = pt;
            pt3d.originatingViews.insert({img1, match.queryIdx});
            pt3d.originatingViews.insert({img2, match.trainIdx});
            pc.addPoint(pt3d);
        }
        return pc;
    }
};

#endif //SFM_MIDPOINT_TRIANGULATION_H
