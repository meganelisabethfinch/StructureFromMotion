//
// Created by Megan Finch on 13/02/2022.
//

#ifndef SFM_TRIANGULATION_UTIL_H
#define SFM_TRIANGULATION_UTIL_H

#include "./headers/point_cloud.h"
#include "./headers/pose.h"
#include "./headers/camera.h"

class Triangulator {
public:
    virtual PointCloud triangulateImages(ImageID img1, ImageID img2,
            Camera& cam1, Camera& cam2,
            Features& features1, Features& features2,
            Matching2& matching,
            Pose& pose1, Pose& pose2) = 0;
};

#endif //SFM_TRIANGULATION_UTIL_H
