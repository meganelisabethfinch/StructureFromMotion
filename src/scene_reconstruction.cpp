//
// Created by Megan Finch on 28/12/2021.
//

#include <headers/scene_reconstruction.h>
#include <fstream>

#include <opencv2/calib3d.hpp>

SceneReconstruction::SceneReconstruction(std::vector<Image> &mImages,
                                         std::vector<Camera> &mCameras,
                                         std::vector<Features> &mImageFeatures,
                                         Matches &mFeatureMatchMatrix)
                                         : _mImages(mImages), _mCameras(mCameras), _mImageFeatures(mImageFeatures), _mFeatureMatchMatrix(mFeatureMatchMatrix)
{

}

bool SceneReconstruction::initialise(ImageID baseline1, ImageID baseline2) {
    // Clear existing state
    _registeredImages.clear();
    _mCameraPoses.clear();
    _pointCloud.clear();

    // Recover pose
    if (baseline1 == baseline2) {
        return false;
    } else if (baseline1 > baseline2) {
        // Swap over so that baseline1 < baseline2 always
        ImageID tmp = baseline1;
        baseline1 = baseline2;
        baseline2 = tmp;
    }

    Matching2 prunedMatching;
    cv::Matx34d proj1;
    cv::Matx34d proj2;

    bool success = recoverPose(_mCameras[baseline1], _mCameras[baseline2],
                               _mImageFeatures[baseline1], _mImageFeatures[baseline2],
                               _mFeatureMatchMatrix.GetMatchingBetween(baseline1, baseline2),
                               prunedMatching,
                               proj1, proj2);

    auto pose1 = Pose(proj1);
    auto pose2 = Pose(proj2);
    // _mFeatureMatchMatrix[baseline1][baseline2] = prunedMatching;

    success = triangulateViews(baseline1, baseline2,
                               _mCameras[baseline1], _mCameras[baseline2],
                               _mImageFeatures[baseline1], _mImageFeatures[baseline2],
                               prunedMatching,
                               pose1, pose2,
                               _pointCloud);


    // Save recovered poses

    // Save triangulated points

    // Register images
    _registeredImages.insert(baseline1);
    _registeredImages.insert(baseline2);

    return true;
}

void SceneReconstruction::toColmapFile(std::string filename) {
    // TODO
}

void SceneReconstruction::toPlyFile(std::string filename) {
    std::cout << "Converting point cloud to .PLY file." << std::endl;

    std::ofstream file (filename);
    file << "ply" << std::endl;
    file << "format ascii 1.0" << std::endl;
    file << "element vertex " << _pointCloud.size() << std::endl;
    file << "property float x" << std::endl;
    file << "property float y" << std::endl;
    file << "property float z" << std::endl;
    // TODO: add colour
    file << "end_header" << std::endl;

    for (const auto& point3D : _pointCloud) {
        file << point3D.pt.x << " ";
        file << point3D.pt.y << " ";
        file << point3D.pt.z << std::endl;
    }

    file.close();
}

bool SceneReconstruction::recoverPose(Camera &cam1, Camera &cam2,
                                      Features &features1, Features &features2,
                                      Matching2 &matching,
                                      Matching2 &prunedMatching,
                                      cv::Matx34d &pose1,
                                      cv::Matx34d &pose2)
{
    // Get two arrays of matching points
    auto points1 = features1.GetPointsFromMatches(matching, true);
    auto points2 = features2.GetPointsFromMatches(matching, false);

    // TODO: for now, assumes cam1.K == cam2.K
    // but this is not always true
    double focal = cam1.getFocalLength();
    auto pp = cam1.getCentre();

    // Find essential matrix
    cv::Mat E, R, t;
    cv::Mat mask;
    E = cv::findEssentialMat(points1, points2, focal, pp, cv::RANSAC, 0.999, 1.0, mask);

    cv::recoverPose(E, points1, points2, R, t, focal, pp, mask);

    // Set pose1 and pose2
    pose1 = cv::Matx34d::eye();
    pose2 = cv::Matx34d(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0),
                        R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1),
                        R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2));

    // Populate pruned matches
    prunedMatching.clear();
    for (size_t i = 0; i < mask.rows; i++) {
        if (mask.at<uchar>(i)) {
            prunedMatching.push_back(matching[i]);
        }
    }

    std::cout << "Pruned matching: " << prunedMatching.size() << " of " << matching.size() << " matches kept." << std::endl;

    return true;
}

bool SceneReconstruction::triangulateViews(ImageID img1, ImageID img2,
                                           Camera &cam1, Camera &cam2,
                                           Features &features1, Features &features2,
                                           Matching2 &matching,
                                           Pose &pose1, Pose &pose2,
                                           PointCloud& pointCloud)
{
    // Get two arrays of matching points
    auto points1 = features1.GetPointsFromMatches(matching, true);
    auto points2 = features2.GetPointsFromMatches(matching, false);

    cv::Mat normalisedPoints1;
    cv::Mat normalisedPoints2;
    cv::undistortPoints(points1, normalisedPoints1, cam1.getCameraMatrix(), cv::Mat());
    cv::undistortPoints(points2, normalisedPoints2, cam2.getCameraMatrix(), cv::Mat());

    cv::Mat points3dHomogenous;
    cv::triangulatePoints(pose1.getProjectionMatrix(), pose2.getProjectionMatrix(), normalisedPoints1, normalisedPoints2, points3dHomogenous);

    cv::Mat points3d;
    cv::convertPointsFromHomogeneous(points3dHomogenous.t(), points3d);

    // TODO: reprojection errors

    // Add points to point cloud
    for (size_t i = 0; i < points3d.rows; i++) {
        Point3DInMap p;
        p.pt = cv::Point3d(points3d.at<float>(i, 0), points3d.at<float>(i,1), points3d.at<float>(i,2));

        // p.originatingViews[img1] =

        pointCloud.push_back(p);
    }


    return true;
}
