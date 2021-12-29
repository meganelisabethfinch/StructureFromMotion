//
// Created by Megan Finch on 28/12/2021.
//

#ifndef SFM_UTIL_H
#define SFM_UTIL_H

#include <vector>
#include <opencv2/core/types.hpp>
#include "camera.h"
#include "features.h"
#include "pose.h"
#include "point_cloud.h"

class SFMUtilities {
public:
    static bool PassesLoweRatioTest(const std::vector<cv::DMatch>& match);

    static Pose recoverPose(Camera &cam1, Camera &cam2,
                            Features &features1, Features &features2,
                            Matching2 &matching,
                            Matching2 &prunedMatching);

    static PointCloud triangulateViews(ImageID img1, ImageID img2,
            Camera& cam1, Camera& cam2,
            Features& features1, Features& features2,
            Matching2& matching,
            Pose& pose1, Pose& pose2);
};

#endif //SFM_UTIL_H
