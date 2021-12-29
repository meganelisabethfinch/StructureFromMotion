//
// Created by Megan Finch on 28/12/2021.
//

#ifndef SFM_SCENE_RECONSTRUCTION_H
#define SFM_SCENE_RECONSTRUCTION_H

#include <set>
#include "imagecollection.h"
#include "point_cloud.h"
#include "pose.h"

class SceneReconstruction {
private:
    std::vector<Image>& mImages;
    std::vector<Pose> _cameraPoses;
    std::set<ImageID> _registeredImages;
    PointCloud _pointCloud;

    // TODO: remove these?
    std::vector<Camera>& mCameras;
    std::vector<Features>& mImageFeatures;
    Matches& mFeatureMatchMatrix;

public:
    SceneReconstruction(std::vector<Image>& mImages,
                        std::vector<Camera>& mCameras,
                        std::vector<Features>& mImageFeatures,
                        Matches& mFeatureMatchMatrix);

    void initialise(ImageID baseline1, ImageID baseline2);

    void toColmap();

    void toPly();
};
#endif //SFM_SCENE_RECONSTRUCTION_H
