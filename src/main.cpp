#include <headers/types.h>
#include <headers/cli_util.h>
#include <headers/imagecollection.h>
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

    std::cout << "----------- Find Matches -----------" << std::endl;
    auto matcher = CLIUtilities::CreateMatcher(args.matcherType);


    return 0;
}
