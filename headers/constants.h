//
// Created by Megan Finch on 23/12/2021.
//

#ifndef SFM_CONSTANTS_H
#define SFM_CONSTANTS_H
#include "types.h"

const double LOWE_RATIO = 0.6;
const double FOCAL_LENGTH = 2500;

const DetectorType DEFAULT_DETECTOR = DetectorType::SIFT;
const MatcherType DEFAULT_MATCHER = MatcherType::FLANNBASED;

#endif //SFM_CONSTANTS_H
