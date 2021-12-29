//
// Created by Megan Finch on 23/12/2021.
//

#ifndef SFM_POINTCLOUD_H
#define SFM_POINTCLOUD_H

#include <map>

struct Point3DInMap {
    cv::Point3d pt;
    std::map<ImageID, int> originatingViews;
};

typedef std::vector<Point3DInMap> PointCloud;

#endif //SFM_POINTCLOUD_H
