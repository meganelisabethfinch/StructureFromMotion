//
// Created by Megan Finch on 28/12/2021.
//

#include <headers/matches.h>
#include <headers/features.h>
#include <iostream>

Matches::Matches(const cv::Ptr<cv::DescriptorMatcher>& matcher, std::vector<Features>& mFeatures) {
    matrix.resize(mFeatures.size(), std::vector<Matching2>(mFeatures.size()));

    for (size_t i = 0; i < mFeatures.size() - 1; i++) {
        for (size_t j = i + 1; j < mFeatures.size(); j++) {
            std::cout << "Finding matches between images " << i << " and " << j << std::endl;
            mFeatures[i].findMatchesWith(matcher, mFeatures[j], matrix[i][j]);
        }
    }
}

Matching2& Matches::getMatchingBetween(const ImageID i, const ImageID j) {
    // Always index by smaller number first
    const ImageID leftID = (i < j) ? i : j;
    const ImageID rightID = (i < j) ? j : i;
    return matrix[leftID][rightID];
}

void Matches::prune(ImageID i, ImageID j, cv::Mat& mask) {
    if (j > i) {
        ImageID tmp = i;
        i = j;
        j = tmp;
    }

    auto matching = matrix[i][j];
    Matching2 prunedMatching;

    for (size_t i = 0; i < mask.rows; i++) {
        if (mask.at<uchar>(i)) {
            prunedMatching.push_back(matching[i]);
        }
    }

    matrix[i][j] = prunedMatching;
}

size_t Matches::size() {
    return matrix.size();
}

Matches::Matches() = default;
