//
// Created by Megan Finch on 13/02/2022.
//

#ifndef SFM_MIDPOINT_TRIANGULATION_H
#define SFM_MIDPOINT_TRIANGULATION_H

#include "triangulator.h"

class MidpointTriangulator : public Triangulator {
    /* Triangulates a 3D point for which is the midpoint of the shortest line segment between two lines
     *
     * @param centres - one point located on each of N lines
     * @param directions - one vector representing the direction of each of N lines
     * @return - the midpoint
     */
    static cv::Point3d triangulatePoint(const std::vector<cv::Vec3d>& centres,
                                 const std::vector<cv::Vec3d>& directions) {
        double min_dist = std::numeric_limits<double>::max();
        cv::Vec3d min_midpoint(0,0,0);
        size_t size_pts = centres.size();

        // Over all pairs of matching points
        for (size_t i = 0; i < size_pts - 1; i++) {
            cv::Vec3d q1 = centres[i];
            cv::Vec3d v1 = directions[i];
            for (size_t j = i + 1; j < size_pts; j++) {
                cv::Vec3d q2 = centres[j];
                cv::Vec3d v2 = directions[j];

                cv::Vec3d vcross = v1.cross(v2);
                double denom = 1.0 / vcross.dot(vcross);

                double lambda1 = ((q2 - q1).cross(v2)).dot(vcross) * denom;
                double lambda2 = ((q2 - q1).cross(v1)).dot(vcross) * denom;

                cv::Vec3d p1 = q1 + lambda1 * v1;
                cv::Vec3d p2 = q2 + lambda2 * v2;
                cv::Vec3d midpoint = (p1 + p2) * 0.5;

                cv::Vec3d d = p1 - p2;
                double dist = sqrt(d.dot(d));
                cv::Vec3d cd = q1 - q2;
                double centre_dist = sqrt(cd.dot(cd));

                if (dist * centre_dist < min_dist) {
                    // TODO: why this formulation of distance?
                    min_dist = dist * centre_dist;
                    min_midpoint = midpoint;
                }
            }
        }
        return { min_midpoint };
    }

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

public:
    PointCloud triangulateImages(ImageID img1, ImageID img2,
                                 Camera& cam1, Camera& cam2,
                                 Features& features1, Features& features2,
                                 Matching2& matching,
                                 Pose& pose1, Pose& pose2) override
    {
        PointCloud pc;

        // Implements equation 6.14 from MVG in Computer Vision (Hartley & Zisserman)
        // TODO: test rotation matrix is correct!
        cv::Matx33d M1 = cam1.getCameraMatrix() * pose1.getRotationMatrix();
        cv::Matx33d M2 = cam2.getCameraMatrix() * pose2.getRotationMatrix();
        cv::Matx31d t1 = cam1.getCameraMatrix() * pose1.getTranslationVector();
        cv::Matx31d t2 = cam1.getCameraMatrix() * pose2.getTranslationVector();

        for (auto& match : matching) {
            // Get q1, q2 (points on the lines L1 and L2, respectively)
            cv::Matx31d res = - M1.inv() * t1;
            cv::Vec3d q1 = { res(0), res(1), res(2) };
            res = - M2.inv() * t2;
            cv::Vec3d q2 = { res(0), res(1), res(2) };

            // Get image points
            cv::Point2d pt1 = features1.getPoint(match.trainIdx);
            cv::Point2d pt2 = features2.getPoint(match.queryIdx);

            // Get directions
            cv::Vec3d pt1_hom = { pt1.x, pt1.y, 1 };
            cv::Vec3d v1 = M1.inv() * pt1_hom;

            cv::Vec3d pt2_hom = { pt2.x, pt2.y, 1 };
            cv::Vec3d v2 = M2.inv() * pt2_hom;

            // Do the triangulation
            auto pt = triangulatePoint(q1, q2, v1, v2);

            // Add to point cloud
            Point3DInMap pt3d;
            pt3d.pt = pt;
            pt3d.originatingViews.insert({img1, match.trainIdx});
            pt3d.originatingViews.insert({img2, match.queryIdx});
        }
        return pc;
    }
};

#endif //SFM_MIDPOINT_TRIANGULATION_H
