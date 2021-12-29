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
