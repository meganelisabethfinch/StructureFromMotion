//
// Created by Megan Finch on 13/02/2022.
//

#ifndef SFM_LINEAR_TRIANGULATION_H
#define SFM_LINEAR_TRIANGULATION_H

#include "triangulator.h"

class LinearTriangulator : public Triangulator {
    PointCloud triangulateImages(ImageID img1, ImageID img2,
               Camera& cam1, Camera& cam2,
               Features& features1, Features& features2,
               Matching2& matching,
               Pose& pose1, Pose& pose2) override;
};

#endif //SFM_LINEAR_TRIANGULATION_H
