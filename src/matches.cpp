//
// Created by Megan Finch on 28/12/2021.
//

#include <headers/constants.h>
#include <headers/matches.h>
#include <headers/features.h>
#include <iostream>

Matches::Matches(const cv::Ptr<cv::DescriptorMatcher>& matcher, std::vector<Features>& mFeatures) {
    matrix.resize(mFeatures.size(), std::vector<Matching2>(mFeatures.size()));

    for (size_t i = 0; i < mFeatures.size() - 1; i++) {
        for (size_t j = i + 1; j < mFeatures.size(); j++) {
            if (DEFAULT_DEBUG >= DebugLevel::VERBOSE) {
                std::cout << "Finding matches between images " << i << " and " << j << std::endl;
            }
            mFeatures[i].findMatchesWith(matcher, mFeatures[j], matrix[i][j]);
        }
    }
}

Matching2& Matches::get(const ImagePair imagePair) {
    // Always index by smaller number first
    return matrix[imagePair.left][imagePair.right];
}

void Matches::prune(const ImagePair& imagePair, cv::Mat& mask) {
    auto i = imagePair.left;
    auto j = imagePair.right;

    Matching2 matching = matrix[i][j];
    Matching2 prunedMatching;

    for (int i = 0; i < mask.rows; i++) {
        if (mask.at<uchar>(i)) {
            prunedMatching.push_back(matching.at(i));
        }
    }

    matrix[i][j] = prunedMatching;

    if (DEFAULT_DEBUG >= DebugLevel::VERBOSE) {
        std::cout << "Pruned matching: " << prunedMatching.size() << " of " << matching.size() << " matches kept."
                  << std::endl;
    }
}

size_t Matches::size() {
    return matrix.size();
}

Matches::Matches() = default;
