//
// Created by Megan Finch on 23/12/2021.
//

#ifndef SFM_CLI_H
#define SFM_CLI_H

#include "types.h"
#include "cli_args.h"

#include <opencv2/features2d.hpp>

class CLIUtilities {
public:
    static bool ParseInputs(int argc, char** argv, Args& args);

    static cv::Ptr<cv::FeatureDetector> CreateDetector(DetectorType type) {
        switch(type) {
            case DetectorType::SIFT:
                return cv::SIFT::create();
            case DetectorType::ORB:
                return cv::ORB::create(5000);
        }
    }

    static cv::Ptr<cv::DescriptorMatcher> CreateMatcher(MatcherType type) {
        return cv::DescriptorMatcher::create(type);
    }
};

#endif //SFM_CLI_H
