//
// Created by Megan Finch on 23/12/2021.
//

#ifndef SFM_POINTCLOUD_H
#define SFM_POINTCLOUD_H

#include <map>
#include "types.h"

struct Point3DInMap {
    cv::Point3d pt;
    std::map<ImageID, int> originatingViews;
};

class PointCloud {
private:
    std::vector<Point3DInMap> map;

public:
    PointCloud();

    Point3DInMap operator[](size_t i);

    std::vector<Point3DInMap>::iterator begin();
    std::vector<Point3DInMap>::iterator end();
};

#endif //SFM_POINTCLOUD_H
