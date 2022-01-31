//
// Created by Megan Finch on 30/01/2022.
//

#ifndef SFM_CLI_ARGS_H
#define SFM_CLI_ARGS_H

#include "image_pair.h"

struct Args {
    std::string inputImageDir;
    std::string outputDir;

    ImagePair* baselinePair;

    DetectorType detectorType;
    MatcherType matcherType;
};

#endif //SFM_CLI_ARGS_H
