//
// Created by Megan Finch on 29/12/2021.
//

#include <headers/point_cloud.h>
#include <vector>

PointCloud::PointCloud() {}

Point3DInMap PointCloud::operator[](size_t i) {
    return map[i];
}

std::vector<Point3DInMap>::iterator PointCloud::begin() {
    return map.begin();
}

std::vector<Point3DInMap>::iterator PointCloud::end() {
    return map.end();
}
