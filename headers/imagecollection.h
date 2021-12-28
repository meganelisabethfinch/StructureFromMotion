//
// Created by Megan Finch on 23/12/2021.
//

#ifndef SFM_IMAGECOLLECTION_H
#define SFM_IMAGECOLLECTION_H

#include <string>
#include <opencv2/core/mat.hpp>
#include "image.h"
#include "types.h"
#include "intrinsic.h"

class ImageCollection {
private:
    std::vector<Image> images;
    Intrinsics intrinsics;
    Features features;
    Matches matches;

public:
    ImageCollection(std::string filepath);
};

#endif //SFM_IMAGECOLLECTION_H
