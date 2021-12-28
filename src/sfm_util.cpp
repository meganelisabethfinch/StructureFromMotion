//
// Created by Megan Finch on 28/12/2021.
//

#include <headers/sfm_util.h>
#include "headers/constants.h"

bool SFMUtilities::PassesLoweRatioTest(const std::vector<cv::DMatch> &match) {
    return match.size() == 2 && static_cast<double>(match[0].distance) < static_cast<double>(match[1].distance) * LOWE_RATIO;
}
