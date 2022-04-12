//
// Created by Megan Finch on 30/01/2022.
//

#ifndef SFM_CLI_ARGS_H
#define SFM_CLI_ARGS_H

#include <set>
#include "image_pair.h"

struct Args {
    std::string inputImageDir;
    std::string outputDir;
    std::set<OutputType> outputTypes;

    bool calibrationOn;
    std::string calibrationDir;

    bool useHomographyOrdering;
    ImagePair baselinePair = ImagePair(0,1);

    DetectorType detectorType;
    MatcherType matcherType;
    TriangulatorType triangulatorType;
    BundleAdjusterType bundleAdjusterType;
    LossType lossType;

    std::set<FilterType> filterTypes;
};

#endif //SFM_CLI_ARGS_H
