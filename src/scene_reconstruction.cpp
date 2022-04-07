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
                                         const cv::Ptr<BundleAdjuster>& bundleAdjuster,
                                         std::vector<cv::Ptr<Filter>>& filters)
        : _mImages(mImages), _mCameras(mCameras),
        _mImageFeatures(mImageFeatures), _mFeatureMatchMatrix(mFeatureMatchMatrix),
        _triangulator(triangulator), _bundleAdjuster(bundleAdjuster),
        _filters(filters)
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
                                         const cv::Ptr<BundleAdjuster>& bundleAdjuster,
                                         std::vector<cv::Ptr<Filter>>& filters)
                                         : _mImages(mImages), _mCameras(mCameras),
                                         _mImageFeatures(mImageFeatures), _mFeatureMatchMatrix(mFeatureMatchMatrix),
                                         _triangulator(triangulator), _bundleAdjuster(bundleAdjuster),
                                        _filters(filters)
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
            std::cout << "Initialised reconstruction with " << pc.size() << " points." << std::endl;
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
                std::cout << "Merging in " << pc.size() << " triangulated points from images " << ip.left << " and " << ip.right << "." << std::endl;
                _pointCloud.mergePoints(pc, _mFeatureMatchMatrix);
                anyViewSuccess = true;
            } catch (std::runtime_error &e) {
                std::cout << "Cannot triangulate points between images " << ip.left << " and " << ip.right << ": ";
                std::cout << e.what() << std::endl;
            }
        }

        _mGoodViews.insert(imageId);
        if (anyViewSuccess) {
            applyFilters();
            adjustBundle();
        }

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

bool SceneReconstruction::applyFilters() {
    for (auto& filter : _filters) {
        filter->filterOutliers(_pointCloud);
    }
    return true;
}

void SceneReconstruction::toPlyFile(const std::string& pointCloudFile, const std::string& cameraFile) {
    _pointCloud.toPlyFile(pointCloudFile, _mImageFeatures, _mImages);

    // Save camera polygons
    std::ofstream cameras_file(cameraFile);
    cameras_file
        << "ply" << std::endl
        << "format ascii 1.0" << std::endl
        << "element vertex " << (_mCameraPoses.size() * 4) << std::endl
        << "property float x " << std::endl
        << "property float y " << std::endl
        << "property float z " << std::endl
        //<< "element edge " << (_mCameraPoses.size() * 3) << std::endl
        // << "property list uchar int vertex_index" << std::endl
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

    cameras_file.close();
}

void SceneReconstruction::outputToFiles(const std::string& outputDirectory, std::set<OutputType> outputTypes){
    for (auto type : outputTypes) {
        std::string filename;
        std::string filename2;
        switch (type) {
            case OutputType::PLY_POINT_CLOUD:
                filename.append(outputDirectory).append("point_cloud.ply");
                _pointCloud.toPlyFile(filename, _mImageFeatures, _mImages);
                break;
            case OutputType::PLY_CAMERAS:
                // TODO: output cameras only here
                filename.append(outputDirectory).append("cameras.ply");
                this->toPlyFile("point_cloud.ply", filename);
                break;
            case OutputType::PCD_POINT_CLOUD:
                filename.append(outputDirectory).append("point_cloud.pcd");
                _pointCloud.toPCDFile(filename, _mImageFeatures, _mImages);
                break;
            case OutputType::VTK_MESH:
                filename.append(outputDirectory).append("mesh.vtk");
                filename2.append(outputDirectory).append("cloud_with_normals.pcd");
                _pointCloud.toVTKFile(filename, filename2);
                break;
        }
    }
}



