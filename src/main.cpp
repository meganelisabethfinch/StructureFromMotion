#include <headers/types.h>
#include <headers/cli.h>
#include <headers/imagecollection.h>
#include <iostream>

int main(int argc, char** argv) {
    std::cout << "----------- Open images ------------" << std::endl;
    Args args;

    if (not CLIUtilities::ParseInputs(argc, argv, args)) {
        return -1;
    }

    ImageCollection images(args.inputImageDir);

    std::cout << "--------- Extract features ---------" << std::endl;
    auto detector = CLIUtilities::CreateDetector(args.detectorType);

    return 0;
}
