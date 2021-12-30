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

size_t PointCloud::size() const {
    return map.size();
}

void PointCloud::addPoint(const Point3DInMap& pt) {
    map.push_back(pt);
}
