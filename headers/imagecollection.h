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
#include "intrinsic.h"
#include "features.h"
#include "matches.h"

class ImageCollection {
private:
    std::vector<Image> mImages;
    std::vector<Intrinsic> mIntrinsics;
    std::vector<Features> mImageFeatures;
    Matches mFeatureMatchMatrix;

public:
    explicit ImageCollection(std::string filepath);

    void ExtractFeatures(const cv::Ptr<cv::FeatureDetector>& detector);

    bool FindMatches(const cv::Ptr<cv::DescriptorMatcher>& matcher);

    [[nodiscard]] size_t size() const;
};

#endif //SFM_IMAGECOLLECTION_H
