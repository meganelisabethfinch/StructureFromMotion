//
// Created by Megan Finch on 23/12/2021.
//

#include <headers/cli_util.h>
#include "headers/constants.h"
#include "headers/image_pair.h"

#include <unistd.h>
#include <iostream>
#include <getopt.h>

#define no_argument 0
#define required_argument 1
#define optional_argument 2

bool CLIUtilities::ParseInputs(int argc, char** argv, Args& args) {
    static struct option long_options[] = {
         // These options set a flag
        {"remove_statistical_outliers", no_argument, &args.sorArgs.enableSOR, 1},
        {"remove_radial_outliers", no_argument, &args.rorArgs.enableROR, 1},
        // These options don't set a flag
        {"bundle_adjuster", required_argument, 0, 'a'},
        {"baseline", required_argument, 0, 'b'},
        {"feature_detector", required_argument, 0, 'd'},
        {"input", required_argument, 0, 'i'},
        {"output", required_argument, 0, 'o'},
        {"triangulator", required_argument, 0, 't'},
        {0,0,0,0},
    };

    // Set defaults before parsing
    args.useHomographyOrdering = DEFAULT_USE_HOMOGRAPHY_ORDERING;
    args.detectorType = DEFAULT_DETECTOR;
    args.matcherType = DEFAULT_MATCHER;
    args.triangulatorType = DEFAULT_TRIANGULATOR;
    args.bundleAdjusterType = DEFAULT_BUNDLE_ADJUSTER;
    args.sorArgs.enableSOR = DEFAULT_ENABLE_SOR;
    args.rorArgs.enableROR = DEFAULT_ENABLE_ROR;
    std::vector<ImageID> baselines;

    // Parse arguments
    int opt;

    while (1) {
        int option_index = 0;
        opt = getopt_long(argc, argv, "i:o:b:t:a:d:", long_options, &option_index);

        // Detect end of options
        if (opt == -1)
            break;

        switch (opt) {
            case 0: {
                // this case sets a flag
                // do nothing else
                break;
            }
            case 'i': {
                args.inputImageDir = optarg;
                break;
            }
            case 'o': {
                args.outputDir = optarg;
                break;
            }
            case 'b': {
                std::istringstream ss(optarg);
                int x;
                if (!(ss >> x)) {
                    std::cerr << "Invalid number: " << argv[1] << '\n';
                } else if (!ss.eof()) {
                    std::cerr << "Trailing characters after number: " << argv[1] << '\n';
                }

                baselines.push_back(x);
                break;
            }
            case 't': {
                static std::map<std::string, TriangulatorType> const str2triangulator = {
                        {"LINEAR",  TriangulatorType::LINEAR },
                        {"MIDPOINT",TriangulatorType::MIDPOINT }
                };
                if (str2triangulator.contains(optarg)) {
                    args.triangulatorType = str2triangulator.at(optarg);
                } else {
                    std::cerr << "Unrecognised triangulator type: " << optarg << ". Using default type." << std::endl;
                }
                break;
            }
            case 'a': {
                static std::map<std::string, BundleAdjusterType> const str2ba = {
                        {"OFF", BundleAdjusterType::OFF },
                        {"BASIC", BundleAdjusterType::BASIC},
                        {"ZHANG", BundleAdjusterType::ZHANG}
                };
                if (str2ba.contains(optarg)) {
                    args.bundleAdjusterType = str2ba.at(optarg);
                } else {
                    std::cerr << "Unrecognised bundle adjuster type: " << optarg << ". Using default type." << std::endl;
                }
                break;
            }
            case 'd': {
                static std::map<std::string, DetectorType> const str2det = {
                        { "SIFT", DetectorType::SIFT },
                        { "ORB", DetectorType::ORB }
                };
                if (str2det.contains(optarg)) {
                    args.detectorType = str2det.at(optarg);
                } else {
                    std::cerr << "Unrecognised feature detector type: " << optarg << ". Using default type." << std::endl;
                }
                break;
            }
            default: {
                std::cerr << "Unrecognised input option: " << opt << std::endl;
                return false;
            }
        }
    }

    if (baselines.empty()) {
        // Do nothing / use default
    } else if (baselines.size() == 2) {
        args.baselinePair = ImagePair(baselines.at(0), baselines.at(1));
    } else {
        std::cerr << "Invalid number of baselines defined: " << baselines.size() << std::endl;
        return false;
    }

    // TODO: Check all mandatory arguments are defined


    return true;
}

void CLIUtilities::Summary(const Args& args) {
    // TODO: can we print names, not numbers please?
    std::cout << "--------- Summary of Inputs ---------" << std::endl;
    std::cout << "Detector type: " << static_cast<std::underlying_type<DetectorType>::type>(args.detectorType) << std::endl;
    std::cout << "Matcher type: " << args.matcherType << std::endl;
    std::cout << "Triangulator type: " << static_cast<std::underlying_type<TriangulatorType>::type>(args.triangulatorType) << std::endl;
    std::cout << "BA type: " << static_cast<std::underlying_type<BundleAdjusterType>::type>(args.bundleAdjusterType) << std::endl;

    if (args.useHomographyOrdering) {
        std::cout << "Baseline: based on homography inlier ordering." << std::endl;
    } else {
        std::cout << "Baseline: (" << args.baselinePair.left << ", " << args.baselinePair.right << ")" << std::endl;
    }

    std::cout << "Enable statistical outlier removal: " << args.sorArgs.enableSOR << std::endl;
    std::cout << "Enable radial outlier removal: " << args.rorArgs.enableROR << std::endl;
}

cv::Ptr<cv::FeatureDetector> CLIUtilities::CreateDetector(DetectorType type) {
    switch(type) {
        case DetectorType::SIFT:
            return cv::SIFT::create();
        case DetectorType::ORB:
            return cv::ORB::create(5000);
    }
}

cv::Ptr<cv::DescriptorMatcher> CLIUtilities::CreateMatcher(MatcherType type) {
    return cv::DescriptorMatcher::create(type);
}

cv::Ptr<Triangulator> CLIUtilities::CreateTriangulator(TriangulatorType type) {
    return Triangulator::create(type);
}

cv::Ptr<BundleAdjuster> CLIUtilities::CreateBundleAdjuster(BundleAdjusterType type) {
    return BundleAdjuster::create(type);
}