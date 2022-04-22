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
        // cv::Vec3d u_prime = { u.x, u.y, K.getFocalLength() };
        // auto p = P.getRotationMatrix() * u_prime + P.getTranslationVector();

        // 10.
        auto c = K.getCentre();
        cv::Vec3d u_prime = { u.x - c.x, u.y - c.y, K.getFocalLength() };
        auto p = P.getRotationMatrix().inv() * u_prime - P.getTranslationVector();

        // 11.
        // auto c = K.getCentre();
        // cv::Vec3d u_prime = { u.x - c.x, u.y - c.y, K.getFocalLength() };
        // auto p = P.getRotationMatrix() * u_prime + P.getTranslationVector();

        return { p(0), p(1), p(2) };
    }

    static std::pair<cv::Vec3d, cv::Vec3d> backProjectPointToRay(const cv::Point2d& u,
                                                                 const Camera& camera,
                                                                 const Pose& pose) {
        // From MVG
        cv::Matx34d P = camera.getCameraMatrix() * pose.getProjectionMatrix();
        Pose k_pose = Pose(P);

        auto R = k_pose.getRotationMatrix();
        auto t = k_pose.getTranslationVector();

        // Find camera centre - from
        // t = -R * c => -R.inv() * t = c
        auto c_mat = -R.inv() * t;
        // cv::Vec4d c = { c_mat(0), c_mat(1), c_mat(2), 1 };

        /*
        // Sanity checks
        auto P_inv = P.t() * (P * P.t()).inv();
        auto check_pseudo_inv = P * P_inv;
        std::cout << "Should be identity: " << std::endl;
        std::cout << check_pseudo_inv << std::endl;

        auto check2 = P * c;
        std::cout << "Should be zero: " << std::endl;
        std::cout << check2 << std::endl;
         */

        // Find and normalise direction vector
        cv::Vec3d x = { u.x , u.y, 1 };
        auto v = R.inv() * x;

        // Just ignore homogenous coords for now
        // cv::Vec4d dir4 = { dir3(0), dir3(1), dir3(2), 0 };

        return {{c_mat(0), c_mat(1), c_mat(2)}, { v(0), v(1), v(2) } };
    }

    PointCloud triangulateImages(ImageID img1, ImageID img2,
                                 Camera& cam1, Camera& cam2,
                                 Features& features1, Features& features2,
                                 Matching2& matching,
                                 Pose& pose1, Pose& pose2) override
    {
        std::cout << "Triangulating points between images " << img1 << " and " << img2 << std::endl;
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
