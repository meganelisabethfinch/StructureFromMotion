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
    _mCameraPoses.emplace(baseline1, pose1);
    _mCameraPoses.emplace(baseline2, pose2);

    // Save triangulated points
    _pointCloud = pc;

    // Register images
    _registeredImages.insert(baseline1);
    _registeredImages.insert(baseline2);

    return true;
}

bool SceneReconstruction::registerImage(ImageID imageId) {
    if (_registeredImages.contains(imageId)) {
        std::cout << "Image " << imageId << " is already registered." << std::endl;
        return false;
    }

    Image2D3DMatch match2D3D;

    // Scan all 3D points currently in point cloud
    for (const Point3DInMap& cloudPoint : _pointCloud) {
        bool found2DPoint = false;
        // true if we find a 2D point in the new image that corresponds to cloudPoint

        // Scan all originating views for that 3D point
        for (const auto& origViewAndPoint : cloudPoint.originatingViews) {
            // Check for 2D-2D matching via the match matrix
            const ImageID originatingImgId = origViewAndPoint.first;
            const int originatingKeyPointIndex = origViewAndPoint.second;

            // if (originatingImgId != imageId - 1) continue;

            for (const cv::DMatch& m : _mFeatureMatchMatrix.GetMatchingBetween(originatingImgId, imageId)) {
                int matching2DPoint = -1;

                if (originatingImgId < imageId) {
                    // originatingImg is query, image being registered is train
                    if (m.queryIdx == originatingKeyPointIndex) {
                        matching2DPoint = m.trainIdx;
                    }
                } else {
                    // image being registered is query, originatingImg is train
                    if (m.trainIdx == originatingKeyPointIndex) {
                        matching2DPoint = m.queryIdx;
                    }
                }

                if (matching2DPoint > -1) {
                    const Features& newImageFeatures = _mImageFeatures[imageId];
                    match2D3D.points2D.push_back(newImageFeatures.getPoint(matching2DPoint));
                    match2D3D.points3D.push_back(cloudPoint.pt);
                    found2DPoint = true;
                    break; // out of matches loop
                }
            }

            if (found2DPoint) { break; } // out of originatingViews loop
        }
    }

    // Recover camera pose for new image to be registered
    Pose newCameraPose = SFMUtilities::recoverPoseFrom2D3DMatches(_mCameras[imageId], match2D3D);
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
    // file << "property uchar red" << std::endl;
    // file << "property uchar green" << std::endl;
    // file << "property uchar blue" << std::endl;
    // TODO: add colour
    file << "end_header" << std::endl;

    for (const auto& point3D : _pointCloud) {
        file << static_cast<float>(point3D.pt.x) << " ";
        file << static_cast<float>(point3D.pt.y) << " ";
        file << static_cast<float>(point3D.pt.z) << std::endl;
    }

    file.close();
}
