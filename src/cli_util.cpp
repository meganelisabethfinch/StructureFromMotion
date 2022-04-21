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
        // These options don't set a flag
        {"bundle_adjuster", required_argument, 0, 'a'},
        {"baseline", required_argument, 0, 'b'},
        {"calibration", required_argument, 0, 'c'},
        {"feature_detector", required_argument, 0, 'd'},
        {"filter", required_argument, 0, 'f'},
        {"output_format", required_argument, 0, 'g'},
        {"input", required_argument, 0, 'i'},
        {"loss", required_argument, 0, 'l'},
        {"output", required_argument, 0, 'o'},
        {"triangulator", required_argument, 0, 't'},
        {0,0,0,0},
    };

    // Set defaults before parsing
    args.outputDir = "";
    args.calibrationOn = false;
    args.useHomographyOrdering = DEFAULT_USE_HOMOGRAPHY_ORDERING;
    args.detectorType = DEFAULT_DETECTOR;
    args.matcherType = DEFAULT_MATCHER;
    args.triangulatorType = DEFAULT_TRIANGULATOR;
    args.bundleAdjusterType = DEFAULT_BUNDLE_ADJUSTER;
    args.lossType = DEFAULT_LOSS_TYPE;
    std::vector<ImageID> baselines;
    args.outputTypes = { OutputType::PLY_POINT_CLOUD, OutputType::PLY_CAMERAS };
    args.filterTypes = { };

    // Parse arguments
    int opt;

    while (true) {
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
            case 'f': {
                static std::map<std::string, FilterType> const str2filter = {
                        {"STATISTICAL",  FilterType::STATISTICAL },
                        {"RADIAL", FilterType::RADIAL }
                };
                if (str2filter.contains(optarg)) {
                    args.filterTypes.insert(str2filter.at(optarg));
                }
                break;
            }
            case 'g': {
                static std::map<std::string, OutputType> const str2out = {
                        {"PLY_POINT_CLOUD",  OutputType::PLY_POINT_CLOUD },
                        {"PLY_CAMERAS", OutputType::PLY_CAMERAS },
                        {"PCD_POINT_CLOUD", OutputType::PCD_POINT_CLOUD },
                        {"VTK_MESH", OutputType::VTK_MESH},
                        {"TXT_REPORT", OutputType::TXT_REPORT}
                };
                if (str2out.contains(optarg)) {
                    args.outputTypes.insert(str2out.at(optarg));
                }
                break;
            }
            case 'l': {
                static std::map<std::string, LossType> const str2loss = {
                        {"NULL",  LossType::NULL_LOSS },
                        {"HUBER", LossType::HUBER },
                        {"SOFTLONE", LossType::SOFTLONE },
                        {"CAUCHY", LossType::CAUCHY }
                };
                if (str2loss.contains(optarg)) {
                    args.lossType = str2loss.at(optarg);
                }
                break;
            }
            case 'c': {
                args.calibrationOn = true;
                args.calibrationDir = optarg;
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
    return true;
}

void CLIUtilities::InputSummary(const Args& args) {
    // TODO: can we print names, not numbers please?
    std::cout << "--------- Summary of Inputs ---------" << std::endl;
    std::cout << "Images will be read from: " << args.inputImageDir << std::endl;
    std::cout << "Outputs will be written to: " << args.outputDir << std::endl;
    std::cout << "Camera calibration: ";
    if (args.calibrationOn) {
        std::cout << args.calibrationDir << std::endl;
    } else {
        std::cout << "Off." << std::endl;
    }

    std::cout << std::endl;

    std::cout << "Detector type: " << static_cast<std::underlying_type<DetectorType>::type>(args.detectorType) << std::endl;
    std::cout << "Matcher type: " << args.matcherType << std::endl;
    std::cout << "Triangulator type: " << static_cast<std::underlying_type<TriangulatorType>::type>(args.triangulatorType) << std::endl;
    std::cout << "BA type: " << static_cast<std::underlying_type<BundleAdjusterType>::type>(args.bundleAdjusterType) << std::endl;
    std::cout << "Loss type: " << static_cast<std::underlying_type<LossType>::type>(args.lossType) << std::endl;

    std::cout << "Filter types: ";
    if (args.filterTypes.size() == 0) {
        std::cout << "None." << std::endl;
    } else {
        for (auto type: args.filterTypes) {
            std::cout << static_cast<std::underlying_type<FilterType>::type>(type) << ", ";
        }
        std::cout << std::endl;
    }

    if (args.useHomographyOrdering) {
        std::cout << "Baseline: based on homography inlier ordering." << std::endl;
    } else {
        std::cout << "Baseline: (" << args.baselinePair.left << ", " << args.baselinePair.right << ")" << std::endl;
    }
}

void CLIUtilities::FindImageFilenames(const std::string& directory, std::vector<cv::String>& filenames) {
    cv::glob(directory + "/*.png", filenames, false);

    std::vector<cv::String> jpgs;
    cv::glob(directory + "/*.jpg", jpgs, false);
    filenames.insert(filenames.end(), jpgs.begin(), jpgs.end());

    cv::glob(directory + "/*.JPG", jpgs, false);
    filenames.insert(filenames.end(), jpgs.begin(), jpgs.end());

    std::cout << "Read " << filenames.size() << " files from " << directory << std::endl;
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

cv::Ptr<BundleAdjuster> CLIUtilities::CreateBundleAdjuster(BundleAdjusterType type, LossType loss) {
    return BundleAdjuster::create(type, loss);
}

std::vector<cv::Ptr<Filter>> CLIUtilities::CreateFilters(const std::set<FilterType> &types) {
    // TODO: cases depending on type of filter, allowing different parameters to be set
    std::vector<cv::Ptr<Filter>> filters = {};
    filters.reserve(types.size());
    for (auto type : types) {
        filters.push_back(Filter::create(type));
    }
    return filters;
}
