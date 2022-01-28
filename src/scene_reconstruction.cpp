//
// Created by Megan Finch on 28/12/2021.
//

#include <headers/scene_reconstruction.h>
#include <headers/pose.h>
#include <headers/ba_util.h>
#include <headers/constants.h>
#include <fstream>

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

bool SceneReconstruction::initialise() {
    std::map<double, ImagePair> sortedImagePairs = SFMUtilities::SortViewsForBaseline(_mImageFeatures, _mFeatureMatchMatrix);

    for (auto& imagePair : sortedImagePairs) {
        ImageID i = imagePair.second.left;
        ImageID j = imagePair.second.right;
        auto matching2 = _mFeatureMatchMatrix.getMatchingBetween(i,j);
        Matching2 prunedMatching;

        try {
            Pose posei = Pose(cv::Matx34d::eye());
            Pose posej = SFMUtilities::recoverPoseFromMatches(_mCameras.at(i),
                                                              _mCameras.at(j),
                                                              _mImageFeatures.at(i),
                                                              _mImageFeatures.at(j),
                                                              matching2,
                                                              prunedMatching);
            double poseInliersRatio = ((double)prunedMatching.size()) / ((double)matching2.size());

            if (poseInliersRatio < POSE_INLIERS_MINIMAL_RATIO) {
                throw std::runtime_error("Insufficient pose inliers " + std::to_string(poseInliersRatio));
            }

            // TODO: Update mFeatureMatchMatrix with prunedMatching

            PointCloud pc = SFMUtilities::triangulateViews(i, j,
                                           _mCameras.at(i), _mCameras.at(j),
                                           _mImageFeatures.at(i), _mImageFeatures.at(j),
                                           prunedMatching,
                                           posei, posej);

            _pointCloud = pc;
            _mCameraPoses.emplace(i, posei);
            _mCameraPoses.emplace(j, posej);
            _registeredImages.insert(i);
            _registeredImages.insert(j);
            break;
        } catch (std::runtime_error& e) {
            std::cerr << "Stereo view could not be obtained from (" << i << "," << j << "): " << e.what() << std::flush;
            continue;
        }
    }
    return false;
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
                std::cout << "Cannot triangulate points between images " << left << " and " << right << ": ";
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

bool SceneReconstruction::adjustBundle() {
    BundleAdjustmentUtilities::adjustBundle(_pointCloud,
                                            _registeredImages,
                                            _mCameraPoses,
                                            _mCameras,
                                            _mImageFeatures);

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

    // Save camera polygons
    std::ofstream cameras_file("_cameras.ply");
    cameras_file
        << "ply" << std::endl
        << "format ascii 1.0" << std::endl
        << "element vertex " << (_mCameraPoses.size() * 4) << std::endl
        << "property float x " << std::endl
        << "property float y " << std::endl
        << "property float z " << std::endl
        // << "element edge " << (_mCameraPoses.size() * 3) << std::endl
        // << "property int vertex1 " << std::endl
        // << "property int vertex2 " << std::endl
        << "end_header" << std::endl;

    for (const auto& kv : _mCameraPoses) {
        auto pose = kv.second;
        auto rmat = pose.getProjectionMatrix();
        auto tvec = pose.getTranslationVector();

        auto c = cv::Point3d(tvec(0), tvec(1), tvec(2));
        cv::Point3d cx = c + cv::Point3d(rmat(0, 0), rmat(1, 0), rmat(2, 0)) * 0.2;
        cv::Point3d cy = c + cv::Point3d(rmat(0, 1), rmat(1, 1), rmat(2, 1)) * 0.2;
        cv::Point3d cz = c + cv::Point3d(rmat(0, 2), rmat(1, 2), rmat(2, 2)) * 0.2;

        cameras_file << c.x  << " " << c.y  << " " << c.z  << std::endl;
        cameras_file << cx.x << " " << cx.y << " " << cx.z << std::endl;
        cameras_file << cy.x << " " << cy.y << " " << cy.z << std::endl;
        cameras_file << cz.x << " " << cz.y << " " << cz.z << std::endl;
    }

}

