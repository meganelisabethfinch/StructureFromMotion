//
// Created by Megan Finch on 28/12/2021.
//

#ifndef SFM_UTIL_H
#define SFM_UTIL_H

#include <vector>
#include <opencv2/core/types.hpp>

class SFMUtilities {
public:
    static bool PassesLoweRatioTest(const std::vector<cv::DMatch>& match);
};

#endif //SFM_UTIL_H
