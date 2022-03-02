//
// Created by Megan Finch on 24/02/2022.
//

#ifndef SFM_BASIC_BUNDLE_ADJUSTER_H
#define SFM_BASIC_BUNDLE_ADJUSTER_H

#include "bundle_adjuster.h"

#include <ceres/ceres.h>
#include <ceres/problem.h>
#include <ceres/rotation.h>
#include <headers/cost/simple_reprojection_error.h>

namespace BundleAdjustUtils {
    static void initLogging() {
        google::InitGoogleLogging("SFM");
    }

    static std::once_flag initLoggingFlag;
}

using namespace BundleAdjustUtils;

class BasicBundleAdjuster : public BundleAdjuster {
    void adjustBundle(Bundle& bundle) override
    {
        std::call_once(initLoggingFlag, initLogging);
        ceres::Problem problem;

        std::map<ImageID, PoseVector> cameraPoses6d;
        for (auto i : bundle.registeredImages) {
            auto pv = bundle.cameraPoses.at(i).toPoseVector();
            cameraPoses6d[i] = pv;
        }

        double focal = bundle.cameras.at(0).getFocalLength();

        std::vector<cv::Vec3d> points3d(bundle.pointCloud.size());

        for (size_t i = 0; i < bundle.pointCloud.size(); i++) {
            const Point3DInMap& p = bundle.pointCloud[i];
            points3d[i] = cv::Vec3d(p.pt.x, p.pt.y, p.pt.z);

            for (const auto& kv : p.originatingViews) {
                cv::Point2d p2d = bundle.features.at(kv.first).getPoint(kv.second);

                // Subtract centre of projection, since the optimiser doesn't know what it is
                p2d.x -= bundle.cameras.at(kv.first).getCentre().x;
                p2d.y -= bundle.cameras.at(kv.first).getCentre().y;

                ceres::CostFunction* cost_function = SimpleReprojectionError::Create(p2d.x, p2d.y);

                PoseVector& pose = cameraPoses6d[kv.first];

                problem.AddResidualBlock(cost_function,
                                         NULL,
                                         pose.val,
                                         points3d[i].val,
                                         &focal);
            }
        }

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.minimizer_progress_to_stdout = true;
        options.max_num_iterations = 500;
        options.eta = 1e-2;
        options.max_solver_time_in_seconds = 10;
        options.logging_type = ceres::LoggingType::SILENT;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        std::cout << summary.BriefReport() << std::endl;

        if (summary.termination_type != ceres::CONVERGENCE) {
            std::cout << "Bundle adjustment failed to converge." << std::endl;
            return;
        }

        // Update
        updateBundle(bundle,
                     points3d,
                     cameraPoses6d,
                     focal);
    }

    void updateBundle(Bundle& bundle,
                      std::vector<cv::Vec3d>& points3d,
                      std::map<ImageID, PoseVector>& cameraPoses6d,
                      double& focal) {
        // Update optimised focal
        for (auto& camera : bundle.cameras) {
            camera.setFocalLength(focal, focal);
        }

        // Update optimised camera poses
        for (auto& kv : cameraPoses6d) {
            ImageID key = kv.first;
            PoseVector& pv = kv.second;

            auto adjustedPose = Pose(pv); // TODO: ADD CAMERA CENTRE???
            bundle.cameraPoses.at(key) = adjustedPose;
        }

        // Update optimised 3D points
        for (size_t i = 0; i < bundle.pointCloud.size(); i++) {
            bundle.pointCloud.updatePoint(i, points3d[i](0), points3d[i](1), points3d[i](2));
        }
    }
};

#endif //SFM_BASIC_BUNDLE_ADJUSTER_H
