//
// Created by Megan Finch on 28/12/2021.
//

#include <headers/scene_reconstruction.h>
#include <headers/pose.h>
#include <headers/ba_util.h>
#include <headers/constants.h>
#include <fstream>

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
                                         Matches &mFeatureMatchMatrix)
        : _mImages(mImages), _mCameras(mCameras), _mImageFeatures(mImageFeatures), _mFeatureMatchMatrix(mFeatureMatchMatrix)
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
                                         ImagePair& baselinePair)
                                         : _mImages(mImages), _mCameras(mCameras), _mImageFeatures(mImageFeatures), _mFeatureMatchMatrix(mFeatureMatchMatrix)
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

            PointCloud pc = SFMUtilities::triangulateViews(i, j,
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
            _registeredImages.insert(i);
            _registeredImages.insert(j);
            break;
        } catch (std::runtime_error &e) {
            std::cerr << "Stereo view could not be obtained from (" << i << "," << j << "): " << e.what() << std::flush;
            continue;
        }
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

        _mCameraPoses.emplace(imageId, newCameraPose);

        // For each image already registered
        // Triangulate points between that and the new image
        for (const ImageID oldId: _registeredImages) {
            auto ip = ImagePair(imageId, oldId);

            try {
                cv::Mat mask;

                // use essential matrix recovery to prune matches
                auto pose_right = SFMUtilities::recoverPoseFromMatches(_mCameras[ip.left], _mCameras[ip.right],
                                                                       _mImageFeatures[ip.left], _mImageFeatures[ip.right],
                                                                       _mFeatureMatchMatrix.get(ip),
                                                                       mask);
                _mFeatureMatchMatrix.prune(ip, mask);

                auto pc = SFMUtilities::triangulateViews(ip.left, ip.right,
                                                         _mCameras[ip.left], _mCameras[ip.right],
                                                         _mImageFeatures[ip.left], _mImageFeatures[ip.right],
                                                         _mFeatureMatchMatrix.get(ip),
                                                         _mCameraPoses.at(ip.left), _mCameraPoses.at(ip.right));

                // TODO: check triangulation successful
                _pointCloud.mergePoints(pc, _mFeatureMatchMatrix);
            } catch (std::runtime_error &e) {
                std::cout << "Cannot triangulate points between images " << ip.left << " and " << ip.right << ": ";
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

