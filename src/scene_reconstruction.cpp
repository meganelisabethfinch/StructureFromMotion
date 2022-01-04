//
// Created by Megan Finch on 28/12/2021.
//

#include <headers/scene_reconstruction.h>
#include <headers/pose.h>
#include <fstream>

#include <opencv2/calib3d.hpp>
#include <headers/sfm_util.h>

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

    // Recover pose
    if (baseline1 == baseline2) {
        return false;
    } else if (baseline1 > baseline2) {
        // Swap over so that baseline1 < baseline2 always
        ImageID tmp = baseline1;
        baseline1 = baseline2;
        baseline2 = tmp;
    }

    try {
        Matching2 prunedMatching;

        auto pose2 = SFMUtilities::recoverPoseFromMatches(_mCameras[baseline1], _mCameras[baseline2],
                                                          _mImageFeatures[baseline1], _mImageFeatures[baseline2],
                                                          _mFeatureMatchMatrix.getMatchingBetween(baseline1, baseline2),
                                                          prunedMatching);

        auto pose1 = Pose(cv::Matx34d::eye());

        // _mFeatureMatchMatrix[baseline1][baseline2] = prunedMatching;

        auto pc = SFMUtilities::triangulateViews(baseline1, baseline2,
                                                 _mCameras[baseline1], _mCameras[baseline2],
                                                 _mImageFeatures[baseline1], _mImageFeatures[baseline2],
                                                 prunedMatching,
                                                 pose1, pose2);

        // Save recovered poses
        _mCameraPoses.emplace(baseline1, pose1);
        _mCameraPoses.emplace(baseline2, pose2);

        // Save triangulated points
        _pointCloud = pc;

        // Register images
        _registeredImages.insert(baseline1);
        _registeredImages.insert(baseline2);

        return true;
    } catch (std::runtime_error& e) {
        std::cerr << "Failed to initialise reconstruction from baseline images " << baseline1 << " and " << baseline2 << ". Please try an alternative baseline pair." << std::endl;
        return false;
    }
}

bool SceneReconstruction::registerImage(ImageID imageId) {
    std::cout << "--------- Register Image " << imageId << " ---------" << std::endl;
    if (_registeredImages.contains(imageId)) {
        std::cout << "Image " << imageId << " is already registered." << std::endl;
        return false;
    }

    // Get a list of matches between keypoints in this image (2D points) and 3D points in the point cloud
    Image2D3DMatch match2D3D = SFMUtilities::find2D3DMatches(imageId, _mImageFeatures[imageId], _mFeatureMatchMatrix, _pointCloud);

    try {
        // Recover camera pose for new image to be registered
        Pose newCameraPose = SFMUtilities::recoverPoseFrom2D3DMatches(_mCameras[imageId], match2D3D);

        // (Check pose is found OK)
        _mCameraPoses.emplace(imageId, newCameraPose);

        // For each image already registered
        // Triangulate points between that and the new image
        for (const ImageID oldId: _registeredImages) {
            ImageID left = (oldId < imageId) ? oldId : imageId;
            ImageID right = (oldId < imageId) ? imageId : oldId;

            try {
                Matching2 prunedMatching;

                // use essential matrix recovery to prune matches
                auto pose_right = SFMUtilities::recoverPoseFromMatches(_mCameras[left], _mCameras[right],
                                                                       _mImageFeatures[left], _mImageFeatures[right],
                                                                       _mFeatureMatchMatrix.getMatchingBetween(left,
                                                                                                               right),
                                                                       prunedMatching);
                // _mFeatureMatchMatrix[left][right] = prunedMatching;


                auto pc = SFMUtilities::triangulateViews(left, right,
                                                         _mCameras[left], _mCameras[right],
                                                         _mImageFeatures[left], _mImageFeatures[right],
                                                         prunedMatching,
                                                         _mCameraPoses.at(left), _mCameraPoses.at(right));

                // TODO: check triangulation successful
                _pointCloud.mergePoints(pc, _mFeatureMatchMatrix);
            } catch (std::runtime_error &e) {
                std::cout << "Cannot triangulate points between images " << left << " and " << right << " because: ";
                std::cout << e.what() << std::endl;
            }
        }

        _registeredImages.insert(imageId);

        return true;
    } catch (std::runtime_error &e) {
        std::cout << "Cannot register image " << imageId << " because: ";
        std::cout << e.what() << std::endl;
        return false;
    }
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
    file << "property uchar red" << std::endl;
    file << "property uchar green" << std::endl;
    file << "property uchar blue" << std::endl;
    file << "end_header" << std::endl;

    for (const auto& point3D : _pointCloud) {
        auto anyOriginatingView = point3D.originatingViews.begin();
        const ImageID viewIdx = anyOriginatingView->first;
        const int keypointIdx = anyOriginatingView->second;
        cv::Point2d point2D = _mImageFeatures[viewIdx].getPoint(keypointIdx);
        cv::Vec3b pointColour = _mImages[viewIdx].getColourAt(point2D);

        file << static_cast<float>(point3D.pt.x) << " ";
        file << static_cast<float>(point3D.pt.y) << " ";
        file << static_cast<float>(point3D.pt.z) << " ";
        file << (int)pointColour(2) << " ";
        file << (int)pointColour(1) << " ";
        file << (int)pointColour(0) << std::endl;
    }

    file.close();
}
