//
// Created by Megan Finch on 28/12/2021.
//

#ifndef SFM_SCENE_RECONSTRUCTION_H
#define SFM_SCENE_RECONSTRUCTION_H

#include <set>
#include "triangulation/triangulator.h"
#include "bundle-adjustment/bundle_adjuster.h"

#include "image.h"
#include "features.h"
#include "camera.h"
#include "matches.h"
#include "point_cloud.h"
#include "pose.h"
#include "image_pair.h"

class SceneReconstruction {
private:
    std::vector<Image>& _mImages;
    std::map<ImageID, Pose> _mCameraPoses;
    std::set<ImageID> _mDoneViews;
    std::set<ImageID> _mGoodViews;
    PointCloud _pointCloud;

    std::vector<Camera>& _mCameras;
    std::vector<Features>& _mImageFeatures;
    Matches& _mFeatureMatchMatrix;

    cv::Ptr<Triangulator> _triangulator;
    cv::Ptr<BundleAdjuster> _bundleAdjuster;

    bool _removeStatisticalOutliers;
    bool _removeRadialOutliers;

    void initialise(std::vector<ImagePair> baselines);

    bool registerImage(ImageID imageId, Image2D3DMatch& match2D3D);

public:
    SceneReconstruction(std::vector<Image>& mImages,
                        std::vector<Camera>& mCameras,
                        std::vector<Features>& mImageFeatures,
                        Matches& mFeatureMatchMatrix,
                        const cv::Ptr<Triangulator>& triangulator,
                        const cv::Ptr<BundleAdjuster>& bundleAdjuster,
                        bool removeStatisticalOutliers,
                        bool removeRadialOutliers);

    SceneReconstruction(std::vector<Image> &mImages,
                        std::vector<Camera> &mCameras,
                        std::vector<Features> &mImageFeatures,
                        Matches &mFeatureMatchMatrix,
                        ImagePair& baselinePair,
                        const cv::Ptr<Triangulator>& triangulator,
                        const cv::Ptr<BundleAdjuster>& bundleAdjuster,
                        bool removeStatisticalOutliers,
                        bool removeRadialOutliers);

    void registerMoreImages();

    bool registerImage(ImageID imageId);

    bool adjustBundle();

    void toColmapFile(const std::string& filename);

    void toPlyFile(const std::string& pointCloudFile, const std::string& cameraFile);
};
#endif //SFM_SCENE_RECONSTRUCTION_H
