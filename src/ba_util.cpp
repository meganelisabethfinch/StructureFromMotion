//
// Created by Megan Finch on 25/04/2022.
//

#include <headers/ba_util.h>
#include <ceres/problem.h>
#include <headers/cost/simple_reprojection_error.h>
#include <ceres/loss_function.h>
#include <ceres/solver.h>

double BAUtilities::globalReprojectionError(PointCloud& pointCloud,
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
