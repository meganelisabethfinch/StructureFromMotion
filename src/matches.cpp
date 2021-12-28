//
// Created by Megan Finch on 28/12/2021.
//

#include <headers/matches.h>
#include <headers/features.h>

Matches::Matches(const cv::Ptr<cv::DescriptorMatcher>& matcher, std::vector<Features>& mFeatures) {

    for (size_t i = 0; i < mFeatures.size() - 1; i++) {
        for (size_t j = i + 1; j < mFeatures.size(); j++) {
            Matching2 matching2 = mFeatures[i].FindMatchesWith(matcher, mFeatures[j]);
        }
    }

}

Matching2& Matches::GetMatchingBetween(const ImageID i, const ImageID j) {
    // Always index by smaller number first
    if (i < j) {
        return matrix[i][j];
    }
    return matrix[j][i];
}

Matches::Matches() = default;
