//
// Created by Megan Finch on 26/02/2022.
//

#ifndef SFM_BUNDLE_H
#define SFM_BUNDLE_H

#include "point_cloud.h"
#include "pose.h"
#include "camera.h"

struct Bundle {
    PointCloud& pointCloud;
    const std::set<ImageID>& registeredImages;
    std::map<ImageID, Pose>& cameraPoses;
    std::vector<Camera>& cameras;
    std::vector<Features>& features;

    Bundle(PointCloud& pointCloud,
           const std::set<ImageID>& images,
           std::map<ImageID, Pose>& cameraPoses,
           std::vector<Camera>& cameras,
           std::vector<Features>& features)
           : pointCloud(pointCloud),
           registeredImages(images),
           cameraPoses(cameraPoses),
           cameras(cameras),
           features(features)
    {
    };
};

#endif //SFM_BUNDLE_H
