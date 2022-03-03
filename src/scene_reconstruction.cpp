//
// Created by Megan Finch on 28/12/2021.
//

#include <algorithm>

#include <headers/scene_reconstruction.h>
#include <headers/pose.h>
#include <headers/ba_util.h>
#include <headers/constants.h>
#include <fstream>
#include <utility>

#include <headers/sfm_util.h>
#include <headers/image_pair.h>

typedef std::map<unsigned, ImagePair> SortedImageMap;

struct get_second : public std::unary_function<SortedImageMap::value_type, std::string>
{
    ImagePair operator()(const SortedImageMap::value_type& value) const
    {
        return value.second;
    }
};

SceneReconstruction::SceneReconstruction(std::vector<Image> &mImages,
                                         std::vector<Camera> &mCameras,
                                         std::vector<Features> &mImageFeatures,
                                         Matches &mFeatureMatchMatrix,
                                         const cv::Ptr<Triangulator>& triangulator,
                                         const cv::Ptr<BundleAdjuster>& bundleAdjuster)
        : _mImages(mImages), _mCameras(mCameras), _mImageFeatures(mImageFeatures), _mFeatureMatchMatrix(mFeatureMatchMatrix), _triangulator(triangulator), _bundleAdjuster(bundleAdjuster)
{
    std::map<double, ImagePair> imagePairsByHomographyInliers = SFMUtilities::SortViewsForBaseline(_mImageFeatures, _mFeatureMatchMatrix);

    std::vector<ImagePair> orderedImagePairs;
    transform(imagePairsByHomographyInliers.begin(), imagePairsByHomographyInliers.end(), back_inserter(orderedImagePairs), get_second() );

    initialise(orderedImagePairs);
}

SceneReconstruction::SceneReconstruction(std::vector<Image> &mImages,
                                         std::vector<Camera> &mCameras,
                                         std::vector<Features> &mImageFeatures,
                                         Matches &mFeatureMatchMatrix,
                                         ImagePair& baselinePair,
                                         const cv::Ptr<Triangulator>& triangulator,
                                         const cv::Ptr<BundleAdjuster>& bundleAdjuster)
                                         : _mImages(mImages), _mCameras(mCameras),
                                         _mImageFeatures(mImageFeatures), _mFeatureMatchMatrix(mFeatureMatchMatrix),
                                         _triangulator(triangulator), _bundleAdjuster(bundleAdjuster)
{
    std::vector<ImagePair> orderedImagePairs;
    orderedImagePairs.push_back(baselinePair);

    initialise(orderedImagePairs);
}

void SceneReconstruction::initialise(std::vector<ImagePair> baselines) {
    for (auto &pair: baselines) {
        ImageID i = pair.left;
        ImageID j = pair.right;
        std::cout << "Trying baseline (" << i << ", " << j << ")" << std::endl;
        auto matching2 = _mFeatureMatchMatrix.get(pair);

        try {
            cv::Mat mask;
            Pose posei = Pose(cv::Matx34d::eye());
            Pose posej = SFMUtilities::recoverPoseFromMatches(_mCameras.at(i),
                                                              _mCameras.at(j),
                                                              _mImageFeatures.at(i),
                                                              _mImageFeatures.at(j),
                                                              matching2,
                                                              mask);
            // Prune pose outliers
            _mFeatureMatchMatrix.prune(pair, mask);

            double poseInliersRatio = ((double) _mFeatureMatchMatrix.get(pair).size()) / ((double) matching2.size());
            if (poseInliersRatio < POSE_INLIERS_MINIMAL_RATIO) {
                throw std::runtime_error("Insufficient pose inliers " + std::to_string(poseInliersRatio));
            }

            /*
            PointCloud pc = SFMUtilities::triangulateViews(i, j,
                                                           _mCameras.at(i), _mCameras.at(j),
                                                           _mImageFeatures.at(i), _mImageFeatures.at(j),
                                                           _mFeatureMatchMatrix.get(pair),
                                                           posei, posej);
            */
            PointCloud pc = _triangulator->triangulateImages(i, j,
                                                           _mCameras.at(i), _mCameras.at(j),
                                                           _mImageFeatures.at(i), _mImageFeatures.at(j),
                                                           _mFeatureMatchMatrix.get(pair),
                                                           posei, posej);

            // Save recovered poses
            _mCameraPoses.emplace(i, posei);
            _mCameraPoses.emplace(j, posej);

            // Save triangulated points
            _pointCloud = pc;

            // Register images
            _mDoneViews.insert(i);
            _mDoneViews.insert(j);
            _mGoodViews.insert(i);
            _mGoodViews.insert(j);

            // adjustBundle();
            break;
        } catch (std::runtime_error &e) {
            std::cerr << "Stereo view could not be obtained from (" << i << "," << j << "): " << e.what() << std::flush;
            continue;
        }
    }
}

bool SceneReconstruction::registerImage(ImageID imageId) {
    // Get a list of matches between keypoints in this image (2D points) and 3D points in the point cloud
    Image2D3DMatch match2D3D = SFMUtilities::find2D3DMatches(imageId, _mImageFeatures[imageId], _mFeatureMatchMatrix, _pointCloud);

    registerImage(imageId, match2D3D);
    return true;
}

bool SceneReconstruction::registerImage(ImageID imageId, Image2D3DMatch &match2D3D) {
    if (_mDoneViews.contains(imageId)) {
        std::cout << "Image " << imageId << " is already registered." << std::endl;
        return false;
    }

    std::cout << "--------- Register Image " << imageId << " ---------" << std::endl;
    _mDoneViews.insert(imageId);

    try {
        // Recover camera pose for new image to be registered
        Pose newCameraPose = SFMUtilities::recoverPoseFrom2D3DMatches(_mCameras[imageId], match2D3D);
        _mCameraPoses.emplace(imageId, newCameraPose);

        // For each image already registered
        // Triangulate points between that and the new image
        bool anyViewSuccess = false;
        for (const ImageID oldId: _mGoodViews) {
            auto ip = ImagePair(imageId, oldId);

            try {
                cv::Mat mask;

                // use essential matrix recovery to prune matches
                auto pose_right = SFMUtilities::recoverPoseFromMatches(_mCameras[ip.left], _mCameras[ip.right],
                                                                       _mImageFeatures[ip.left], _mImageFeatures[ip.right],
                                                                       _mFeatureMatchMatrix.get(ip),
                                                                       mask);
                _mFeatureMatchMatrix.prune(ip, mask);

                auto pc = _triangulator->triangulateImages(ip.left, ip.right,
                                                             _mCameras[ip.left], _mCameras[ip.right],
                                                             _mImageFeatures[ip.left], _mImageFeatures[ip.right],
                                                             _mFeatureMatchMatrix.get(ip),
                                                             _mCameraPoses.at(ip.left), _mCameraPoses.at(ip.right));

                // If we reach this point, triangulation was successful
                _pointCloud.mergePoints(pc, _mFeatureMatchMatrix);
                anyViewSuccess = true;
            } catch (std::runtime_error &e) {
                std::cout << "Cannot triangulate points between images " << ip.left << " and " << ip.right << ": ";
                std::cout << e.what() << std::endl;
            }
        }

        // TODO: review -- do we need both goodViews and doneViews?
        _mGoodViews.insert(imageId);

        if (anyViewSuccess) {
            adjustBundle();
        }
        // TODO: end review

        return true;
    } catch (std::runtime_error &e) {
        std::cout << "Cannot register image " << imageId << " because: ";
        std::cout << e.what() << std::endl;
        return false;
    }
}

void SceneReconstruction::registerMoreImages() {
    std::cout << "-------- Adding more views ---------" << std::endl;

    while (_mDoneViews.size() != _mImages.size()) {
        // Find all 2D-3D correspondences between unregistered images and current point cloud
        std::map<ImageID, Image2D3DMatch> matches2D3D;
        for (const auto& image : _mImages) {
            auto id = image.id;
            if (_mDoneViews.contains(id)) {
                continue; // skip done views
            }

            matches2D3D[id] = SFMUtilities::find2D3DMatches(id,
                                          _mImageFeatures.at(id),
                                          _mFeatureMatchMatrix,
                                          _pointCloud);
        }

        // Get view with highest number of correspondences
        ImageID bestView;
        size_t bestCount = 0;
        for (const auto& match2D3D : matches2D3D) {
            const size_t count = match2D3D.second.size();
            if (count > bestCount) {
                bestView = match2D3D.first;
                bestCount = count;
            }
        }

        // register best view
        registerImage(bestView, matches2D3D.at(bestView));
    }

}

bool SceneReconstruction::adjustBundle() {
    Bundle bundle(_pointCloud,
                  _mGoodViews,
                  _mCameraPoses,
                  _mCameras,
                  _mImageFeatures);
    _bundleAdjuster->adjustBundle(bundle);
    return true;
}

void SceneReconstruction::toColmapFile(const std::string& filename) {
    // TODO
}

void SceneReconstruction::toPlyFile(const std::string& pointCloudFile, const std::string& cameraFile) {
    _pointCloud.toPlyFile(pointCloudFile, _mImageFeatures, _mImages);
    /*
    std::cout << "Converting point cloud to .PLY file." << std::endl;

    std::ofstream file (pointCloudFile);
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
*/
    // Save camera polygons
    std::ofstream cameras_file(cameraFile);
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


