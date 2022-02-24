//
// Created by Megan Finch on 13/02/2022.
//

#ifndef SFM_TRIANGULATION_UTIL_H
#define SFM_TRIANGULATION_UTIL_H

#include "../point_cloud.h"
#include "../pose.h"
#include "../camera.h"

class Triangulator {
public:
    virtual PointCloud triangulateImages(ImageID img1, ImageID img2,
            Camera& cam1, Camera& cam2,
            Features& features1, Features& features2,
            Matching2& matching,
            Pose& pose1, Pose& pose2) = 0;

    virtual ~Triangulator() = default;

    static cv::Ptr<Triangulator> create(const TriangulatorType& type);
};

#endif //SFM_TRIANGULATION_UTIL_H
