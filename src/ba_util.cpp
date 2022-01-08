//
// Created by Megan Finch on 07/01/2022.
//

#include <headers/ba_util.h>

#include <ceres/ceres.h>
#include <ceres/problem.h>
#include <ceres/rotation.h>

/*
void initLogging() {
    google::InitGoogleLogging("SFM");
}

std::once_flag initLoggingFlag;
*/

void BundleAdjustmentUtilities::adjustBundle(PointCloud &pointCloud,
                                             std::set<ImageID> &registeredImages,
                                             std::map<ImageID, Pose> &cameraPoses,
                                             std::vector<Camera> &cameras,
                                             std::vector<Features> &features)
{
    // std::call_once(initLoggingFlag, initLogging);

    ceres::Problem problem;

    std::map<ImageID, PoseVector> poseVectors;

    for (const auto id : registeredImages) {
        // get rotation and translation vector
        auto pose = cameraPoses.at(id);
        auto t = pose.getTranslationVector();
        auto R = pose.getRotationVector();

        double angleAxis[3];
        ceres::RotationMatrixToAngleAxis<double>(R.t().val, angleAxis);

        poseVectors.emplace(id, PoseVector(
                angleAxis[0], angleAxis[1], angleAxis[2],
                t(0), t(1), t(2)
                ));
    }

    std::vector<cv::Vec3d> points3d(pointCloud.size());
    /*
    for (const auto& point3d : pointCloud) {
        points3d[i] = cv::Vec3d(point3d.pt.x, point3d.pt.y, point3d.pt.z);

        for (const auto& kv : point3d.originatingViews) {
            ImageID imgIdx = kv.first;
            int kpIdx = kv.second;

            cv::Point2d point2d = features.at(imgIdx).getPoint(kpIdx);

            // subtract centre of projection
            point2d.x -= cameras[imgIdx].getCentre().x;
            point2d.y -= cameras[imgIdx].getCentre().y;
            double focal = cameras[imgIdx].getFocalLength();

            ceres::CostFunction* costFunction = SimpleReprojectionError::Create(point2d.x, point2d.y);

            // Each residual block is loss(cost(cameraPose, point3d, focal))
            // .val returns the matrix/vector as a double* (double array)
            problem.AddResidualBlock(costFunction,
                                     nullptr, // loss function
                                     poseVectors.at(imgIdx).val,
                                     points3d[i].val,
                                     &focal);
        }
    }
     */

    /*
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

     if (not (summary.termination_type == ceres::CONVERGENCE)) {
        std::cerr << "Bundle adjustment failed." << std::endl;
        return;
     }

     // TODO: update optimised focal length


     // TODO: update optimised camera poses and 3D points
     */

    // Replace with optimised camera poses
    for (const auto id : registeredImages) {
        auto newPose = Pose(poseVectors.at(id));
        cameraPoses.at(id) = newPose;
    }

    // Replace with optimised points


}

