//
// Created by Megan Finch on 28/12/2021.
//

#ifndef SFM_MATCHES_H
#define SFM_MATCHES_H

#include "features.h"
#include "image_pair.h"

class Matches {
private:
    MatchMatrix matrix;

public:
    Matches(const cv::Ptr<cv::DescriptorMatcher>& matcher, std::vector<Features>& mFeatures);

    Matches(); // TODO: remove this while keeping the compiler happy

    Matching2& get(const ImagePair imagePair);

    void prune(const ImagePair& imagePair, cv::Mat& mask);

    size_t size();
};

#endif //SFM_MATCHES_H
