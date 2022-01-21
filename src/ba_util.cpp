//
// Created by Megan Finch on 07/01/2022.
//

#include <headers/ba_util.h>

#include <ceres/ceres.h>
#include <ceres/problem.h>
#include <ceres/rotation.h>
#include <headers/cost/simple_reprojection_error.h>


void BundleAdjustmentUtilities::adjustBundle(PointCloud &pointCloud,
                                             std::set<ImageID> &registeredImages,
                                             std::map<ImageID, Pose> &cameraPoses,
                                             std::vector<Camera> &cameras,
                                             std::vector<Features> &features)
{
    ceres::Problem problem;

    std::map<ImageID, PoseVector> poseVectors;

    for (const auto id : registeredImages) {
        auto pose = cameraPoses.at(id);
        poseVectors.emplace(id, pose.getPoseVector());
    }

    std::vector<cv::Vec3d> points3d(pointCloud.size());
    std::map<ImageID, double> focals;

    for (int i = 0; i < pointCloud.size(); i++) {
        auto point3d = pointCloud[i];
        points3d[i] = cv::Vec3d(point3d.pt.x, point3d.pt.y, point3d.pt.z);

        for (const auto& kv : point3d.originatingViews) {
            ImageID imgIdx = kv.first;
            int kpIdx = kv.second;

            cv::Point2d point2d = features.at(imgIdx).getPoint(kpIdx);

            // subtract centre of projection
            point2d.x -= cameras[imgIdx].getCentre().x;
            point2d.y -= cameras[imgIdx].getCentre().y;
            // double focal = cameras[imgIdx].getFocalLength();
            focals[imgIdx] = cameras[imgIdx].getFocalLength();

            ceres::CostFunction* costFunction = SimpleReprojectionError::Create(point2d.x, point2d.y);

            // Each residual block is loss(cost(cameraPose, point3d, focal))
            // .val returns the matrix/vector as a double* (double array)
            problem.AddResidualBlock(costFunction,
                                     nullptr, // loss function
                                     poseVectors.at(imgIdx).val,
                                     points3d[i].val,
                                     &focals[imgIdx]);
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

     // Update optimised focal length
     for (auto kv : focals) {
         ImageID imgIdx = kv.first;
         double focal = kv.second;
         cameras[imgIdx].setFocalLength(focal, focal); // assuming fx = fy
     }

    // Replace with optimised camera poses
    for (const auto id : registeredImages) {
        auto newPose = Pose(poseVectors.at(id));
        cameraPoses.at(id) = newPose;
    }

    // Replace with optimised points
    for (size_t i = 0; i < pointCloud.size(); i++) {
        pointCloud[i].setPoint(points3d[i](0),
                               points3d[i](1),
                               points3d[i](2));
    }

    std::cout << "Bundle adjustment completed." << std::endl;
}

std::set<double> BundleAdjustmentUtilities::GetPointsWithCommonViews(PointCloud &pointCloud, size_t N, size_t J) {
    // TODO: Define type with N points and J imageIDs
    std::set<double> tuples;

    for (int i = 0; i < pointCloud.size(); i++) {
        std::vector<int> tuple = { i };
        std::set<ImageID> views = pointCloud[i].getOriginatingViews();

        if (views.size() < J) { continue; }

        _CommonViewsHelper(pointCloud,
                           N - 1,
                           J,
                           views,
                           tuple,
                           tuples);
    }

    return std::set<double>();
}

void BundleAdjustmentUtilities::_CommonViewsHelper(PointCloud &pointCloud,
                                                   size_t pointsNeeded,
                                                   size_t viewsNeeded,
                                                   std::set<ImageID> &commonViews,
                                                   std::vector<int> &points,
                                                   std::set<double> &tuples)
{
    if (pointsNeeded == 0 && commonViews.size() >= viewsNeeded) {
        // Get all *combinations* of (viewsNeeded) views from commonViews
        // And store in tuples

    } else {
        for (int j = points.back() + 1; j < pointCloud.size() - pointsNeeded + 1; j++) {
            std::vector<int> new_tuple = {  }; // [ points..., j]
            new_tuple.insert(new_tuple.end(), points.begin(), points.end());
            new_tuple.push_back(j);
            std::set<ImageID> new_views = pointCloud[j].getOriginatingViews();

            // Filter for views which are common to all points (+ point j)
            std::set<ImageID> intersect;
            std::set_intersection(new_views.begin(), new_views.end(), commonViews.begin(), commonViews.end(),
                                  std::inserter(intersect, intersect.begin()));

            if (intersect.size() < viewsNeeded) { continue; }

            _CommonViewsHelper(pointCloud,
                               pointsNeeded - 1,
                               viewsNeeded,
                               intersect,
                               new_tuple,
                               tuples);
        }
    }
}

