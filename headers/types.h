//
// Created by Megan Finch on 23/12/2021.
//

#ifndef SFM_TYPES_H
#define SFM_TYPES_H

#include <cstddef>
#include <string>

typedef size_t ImageID;

struct Args {
    std::string inputImageDir;
    std::string outputDir;
    // baseline pair
};

#endif //SFM_TYPES_H
