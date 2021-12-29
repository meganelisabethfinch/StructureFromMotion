//
// Created by Megan Finch on 28/12/2021.
//

#include <headers/scene_reconstruction.h>
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

    auto pose2 = SFMUtilities::recoverPose(_mCameras[baseline1], _mCameras[baseline2],
                               _mImageFeatures[baseline1], _mImageFeatures[baseline2],
                               _mFeatureMatchMatrix.GetMatchingBetween(baseline1, baseline2),
                               prunedMatching);

    auto pose1 = Pose(cv::Matx34d::eye());

    // _mFeatureMatchMatrix[baseline1][baseline2] = prunedMatching;

    auto pc = SFMUtilities::triangulateViews(baseline1, baseline2,
                               _mCameras[baseline1], _mCameras[baseline2],
                               _mImageFeatures[baseline1], _mImageFeatures[baseline2],
                               prunedMatching,
                               pose1, pose2);


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
