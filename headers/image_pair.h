//
// Created by Megan Finch on 30/01/2022.
//

#ifndef SFM_IMAGE_PAIR_H
#define SFM_IMAGE_PAIR_H

#include <iostream>
#include "types.h"

struct ImagePair {
    ImageID left{}, right{};

    ImagePair(ImageID id1, ImageID id2) {
        if (id1 < id2) {
            left = id1;
            right = id2;
        } else if (id2 < id1) {
            left = id2;
            right = id1;
        }
    }

    // TODO: override << for this class
    // friend std::ostream& operator<<(std::ostream& os, ImagePair const& ip);
};

/*
std::ostream &operator<<(std::ostream &os, const ImagePair &ip) {
    os << "(" << ip.left << "," << ip.right << ")";
    return os;
}
*/

#endif //SFM_IMAGE_PAIR_H
