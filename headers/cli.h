//
// Created by Megan Finch on 23/12/2021.
//

#ifndef SFM_CLI_H
#define SFM_CLI_H

#include "types.h"

class CLIUtilities {
public:
    static bool ParseInputs(int argc, char** argv, Args& args);
};

#endif //SFM_CLI_H
