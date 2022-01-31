//
// Created by Megan Finch on 07/01/2022.
//

#include <headers/ba_util.h>

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

void BundleAdjustmentUtilities::adjustBundle(PointCloud &pointCloud,
                                             const std::set<ImageID>& registeredImages,
                                             std::map<ImageID, Pose> &cameraPoses,
                                             std::vector<Camera> &cameras,
                                             std::vector<Features> &features)
{
    std::call_once(initLoggingFlag, initLogging);
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

    // Update optimised focal
    // This ampersand has ruined my life.
     for (auto& camera : cameras) {
         camera.setFocalLength(focal, focal);
     }

     // Update optimised camera poses
     for (auto& kv : cameraPoses6d) {
         ImageID key = kv.first;
         PoseVector& pv = kv.second;

         auto adjustedPose = Pose(pv);
         cameraPoses.at(key) = adjustedPose;
     }

     // Update optimised 3D points
     for (size_t i = 0; i < pointCloud.size(); i++) {
         pointCloud.updatePoint(i, points3d[i](0), points3d[i](1), points3d[i](2));
     }
}