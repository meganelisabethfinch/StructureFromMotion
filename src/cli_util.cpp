//
// Created by Megan Finch on 23/12/2021.
//

#include <headers/cli_util.h>
#include "headers/constants.h"

#include <unistd.h>
#include <iostream>

bool CLIUtilities::ParseInputs(int argc, char** argv, Args& args) {
    // Parse arguments
    int opt;
    std::vector<ImageID> baselines;

    while ((opt = getopt(argc, argv, "i:o:b:")) != -1) {
        switch (opt) {
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
            default: {
                std::cerr << "Unrecognised input option: " << opt << std::endl;
                return false;
            }
        }
    }

    // Set defaults for optional arguments
    args.detectorType = DEFAULT_DETECTOR;
    args.matcherType = DEFAULT_MATCHER;

    if (baselines.empty()) {
        // Use defaults
        args.baseline1 = 0;
        args.baseline2 = 1;
    } else if (baselines.size() == 2) {
        args.baseline1 = baselines.at(0);
        args.baseline2 = baselines.at(1);
    } else {
        std::cerr << "Invalid number of baselines defined: " << baselines.size() << std::endl;
        return false;
    }

    // TODO: Check all mandatory arguments are defined


    return true;
}

