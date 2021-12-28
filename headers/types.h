//
// Created by Megan Finch on 23/12/2021.
//

#ifndef SFM_TYPES_H
#define SFM_TYPES_H

#include <cstddef>
#include <string>

typedef size_t ImageID;

enum class DetectorType { SIFT, ORB };
enum class MatcherType { FLANNBASED, BRUTEFORCE };

struct Args {
    std::string inputImageDir;
    std::string outputDir;
    // TODO: add manually specified baseline pair
    DetectorType detectorType;
    MatcherType matcherType;
};

struct Features {

};

struct Matches {

};

#endif //SFM_TYPES_H
