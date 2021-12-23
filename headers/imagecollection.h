//
// Created by Megan Finch on 23/12/2021.
//

#ifndef SFM_IMAGECOLLECTION_H
#define SFM_IMAGECOLLECTION_H

#include <string>
#include "image.h"

class ImageCollection {
private:
    std::vector<Image> images;

public:
    ImageCollection(std::string filepath);
};

#endif //SFM_IMAGECOLLECTION_H
