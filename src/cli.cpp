//
// Created by Megan Finch on 23/12/2021.
//

#include <headers/cli.h>
#include "headers/constants.h"

#include <unistd.h>

bool CLIUtilities::ParseInputs(int argc, char** argv, Args& args) {
    // Set defaults for optional arguments
    args.detectorType = DEFAULT_DETECTOR;
    args.matcherType = DEFAULT_MATCHER;

    // Parse arguments
    int opt;
    while ((opt = getopt(argc, argv, "i:o:")) != -1) {
        switch (opt) {
            case 'i': {
                args.inputImageDir = optarg;
                break;
            }
            case 'o': {
                args.outputDir = optarg;
                break;
            }
            default: {
                return false;
            }
        }
    }

    // TODO: Check all mandatory arguments are defined

    return true;
}

