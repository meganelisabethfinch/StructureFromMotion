//
// Created by Megan Finch on 23/12/2021.
//

#ifndef SFM_IMAGECOLLECTION_H
#define SFM_IMAGECOLLECTION_H

#include <string>
#include <opencv2/core/mat.hpp>
#include <opencv2/features2d.hpp>
#include "image.h"
#include "types.h"
#include "camera.h"
#include "features.h"
#include "matches.h"
#include "scene_reconstruction.h"

class ImageCollection {
private:
    std::vector<Image> mImages;
    std::vector<Camera> mCameras;
    std::vector<Features> mImageFeatures;
    Matches mFeatureMatchMatrix;

public:
    explicit ImageCollection(std::string filepath);

    ImageCollection();

    void ExtractFeatures(const cv::Ptr<cv::FeatureDetector>& detector);

    bool FindMatches(const cv::Ptr<cv::DescriptorMatcher>& matcher);

    [[nodiscard]] size_t size() const;

    Image& getImage(ImageID id);

    void visualiseKeyPoints(ImageID id);

    void visualiseMatches(ImageID i, ImageID j);

    SceneReconstruction reconstruct();
};

#endif //SFM_IMAGECOLLECTION_H
