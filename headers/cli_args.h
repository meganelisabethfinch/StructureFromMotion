//
// Created by Megan Finch on 30/01/2022.
//

#ifndef SFM_CLI_ARGS_H
#define SFM_CLI_ARGS_H

#include "image_pair.h"

// Statistical outlier removal
struct SORArgs {
    int enableSOR;
    int mean;
    double stddev_mult;
};

struct RORArgs {
    int enableROR;
};

struct Args {
    std::string inputImageDir;
    std::string outputDir;

    bool useHomographyOrdering;
    ImagePair baselinePair = ImagePair(0,1);

    DetectorType detectorType;
    MatcherType matcherType;
    TriangulatorType triangulatorType;
    BundleAdjusterType bundleAdjusterType;

    SORArgs sorArgs;
    RORArgs rorArgs;
};

#endif //SFM_CLI_ARGS_H
