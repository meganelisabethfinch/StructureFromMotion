#include <headers/types.h>
#include <headers/cli_util.h>
#include <headers/imagecollection.h>
#include <headers/constants.h>
#include <iostream>

int main(int argc, char** argv) {
    std::cout << "----------- Open Images ------------" << std::endl;
    Args args;

    if (not CLIUtilities::ParseInputs(argc, argv, args)) {
        return -1;
    }

    ImageCollection images(args.inputImageDir);

    std::cout << "--------- Extract Features ---------" << std::endl;
    auto detector = CLIUtilities::CreateDetector(args.detectorType);
    images.ExtractFeatures(detector);

    if (DEFAULT_DEBUG >= DebugLevel::EXAMPLES) {
        images.visualiseKeyPoints(0);
    }

    std::cout << "----------- Find Matches -----------" << std::endl;
    auto matcher = CLIUtilities::CreateMatcher(args.matcherType);
    images.FindMatches(matcher);

    if (DEFAULT_DEBUG >= DebugLevel::EXAMPLES) {
        images.visualiseMatches(0, 1);
    }


    std::cout << "---- Find Baseline Triangulation ---" << std::endl;

    return 0;
}
