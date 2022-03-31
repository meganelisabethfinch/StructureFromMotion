//
// Created by Megan Finch on 23/12/2021.
//

#ifndef SFM_CLI_H
#define SFM_CLI_H

#include "types.h"
#include "cli_args.h"

#include <opencv2/features2d.hpp>
#include "filters/filter.h"
#include "triangulation/triangulator.h"
#include "bundle-adjustment/bundle_adjuster.h"

class CLIUtilities {
public:
    static bool ParseInputs(int argc, char** argv, Args& args);

    static void Summary(const Args& args);

    static cv::Ptr<cv::FeatureDetector> CreateDetector(DetectorType type);

    static cv::Ptr<cv::DescriptorMatcher> CreateMatcher(MatcherType type);

    static cv::Ptr<Triangulator> CreateTriangulator(TriangulatorType type);

    static cv::Ptr<BundleAdjuster> CreateBundleAdjuster(BundleAdjusterType type);

    static std::vector<cv::Ptr<Filter>> CreateFilters(const std::set<FilterType>& types);
};

#endif //SFM_CLI_H
