//
// Created by Megan Finch on 23/12/2021.
//

#include <headers/cli.h>

#include <unistd.h>

bool CLIUtilities::ParseInputs(int argc, char** argv, Args& args) {
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

    // TODO: Check all essential arguments are defined

    return true;
}
