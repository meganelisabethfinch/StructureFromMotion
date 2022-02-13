//
// Created by Megan Finch on 28/12/2021.
//

#ifndef SFM_SCENE_RECONSTRUCTION_H
#define SFM_SCENE_RECONSTRUCTION_H

#include <set>
#include <headers/triangulation/triangulator.h>
#include <headers/bundle-adjustment/bundle_adjuster.h>

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

    cv::Ptr<Triangulator> triangulator;
    cv::Ptr<BundleAdjuster> bundleAdjuster;

    void initialise(std::vector<ImagePair> baselines);

    bool registerImage(ImageID imageId, Image2D3DMatch& match2D3D);

public:
    SceneReconstruction(std::vector<Image>& mImages,
                        std::vector<Camera>& mCameras,
                        std::vector<Features>& mImageFeatures,
                        Matches& mFeatureMatchMatrix);

    SceneReconstruction(std::vector<Image> &mImages,
                        std::vector<Camera> &mCameras,
                        std::vector<Features> &mImageFeatures,
                        Matches &mFeatureMatchMatrix,
                        ImagePair& baselinePair);

    void registerMoreImages();

    bool registerImage(ImageID imageId);

    bool adjustBundle();

    void toColmapFile(std::string filename);

    void toPlyFile(std::string filename);
};
#endif //SFM_SCENE_RECONSTRUCTION_H
