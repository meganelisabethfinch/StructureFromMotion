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
#include "scene_graph.h"

class ImageCollection {
private:
    std::vector<Image> mImages;
    std::vector<Camera> mCameras;
    std::vector<Features> mImageFeatures;
    Matches mFeatureMatchMatrix;

public:
    explicit ImageCollection(const std::string& directory);

    ImageCollection(const std::string& directory, const std::string& calibrationDir);

    void ExtractFeatures(const cv::Ptr<cv::FeatureDetector>& detector);

    bool FindMatches(const cv::Ptr<cv::DescriptorMatcher>& matcher);

    [[nodiscard]] size_t size() const;

    Image& getImage(ImageID id);

    void visualiseKeyPoints(ImageID id);

    void visualiseMatches(ImageID i, ImageID j);

    SceneReconstruction toSceneReconstruction(const cv::Ptr<Triangulator>& triangulator,
                                              const cv::Ptr<BundleAdjuster>& bundleAdjuster,
                                              std::vector<cv::Ptr<Filter>>& filters,
                                              ImagePair& imagePair);

    SceneReconstruction toSceneReconstruction(const cv::Ptr<Triangulator>& triangulator,
                                              const cv::Ptr<BundleAdjuster>& bundleAdjuster,
                                              std::vector<cv::Ptr<Filter>>& filters);

    SceneGraph toSceneGraph();
};

#endif //SFM_IMAGECOLLECTION_H
