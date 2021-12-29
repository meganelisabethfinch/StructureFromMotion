//
// Created by Megan Finch on 28/12/2021.
//

#ifndef SFM_SCENE_RECONSTRUCTION_H
#define SFM_SCENE_RECONSTRUCTION_H

#include <set>
#include "image.h"
#include "features.h"
#include "camera.h"
#include "matches.h"
#include "point_cloud.h"
#include "pose.h"

class SceneReconstruction {
private:
    std::vector<Image>& _mImages;
    std::vector<Pose> _mCameraPoses;
    std::set<ImageID> _registeredImages;
    PointCloud _pointCloud;

    // TODO: remove these?
    std::vector<Camera>& _mCameras;
    std::vector<Features>& _mImageFeatures;
    Matches& _mFeatureMatchMatrix;

    bool recoverPose(Camera& cam1, Camera& cam2,
                     Features& features1, Features& features2,
                     Matching2& matching,
                     Matching2& prunedMatching,
                     cv::Matx34d& pose1, cv::Matx34d& pose2);

    // TODO: pass a struct instead of a long list of parameters
    bool triangulateViews(ImageID img1, ImageID img2,
                          Camera& cam1, Camera& cam2,
                          Features& features1, Features& features2,
                          Matching2& matching,
                          Pose& pose1, Pose& pose2,
                          PointCloud& pointCloud);

public:
    SceneReconstruction(std::vector<Image>& mImages,
                        std::vector<Camera>& mCameras,
                        std::vector<Features>& mImageFeatures,
                        Matches& mFeatureMatchMatrix);

    bool initialise(ImageID baseline1, ImageID baseline2);

    void toColmapFile(std::string filename);

    void toPlyFile(std::string filename);
};
#endif //SFM_SCENE_RECONSTRUCTION_H
