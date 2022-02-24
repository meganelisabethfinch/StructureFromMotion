//
// Created by Megan Finch on 30/01/2022.
//

#ifndef SFM_CLI_ARGS_H
#define SFM_CLI_ARGS_H

#include "image_pair.h"

struct Args {
    std::string inputImageDir;
    std::string outputDir;

    bool useHomographyOrdering;
    ImagePair baselinePair = ImagePair(0,1);

    DetectorType detectorType;
    MatcherType matcherType;
    TriangulatorType triangulatorType;
    BundleAdjusterType bundleAdjusterType;
};

#endif //SFM_CLI_ARGS_H
