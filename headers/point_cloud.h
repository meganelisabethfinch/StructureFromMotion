//
// Created by Megan Finch on 23/12/2021.
//

#ifndef SFM_POINTCLOUD_H
#define SFM_POINTCLOUD_H

#include <map>
#include <set>
#include "types.h"
#include "matches.h"

struct Point3DInMap {
    cv::Point3d pt;
    std::map<ImageID, int> originatingViews;

    std::set<ImageID> getOriginatingViews() {
        std::set<ImageID> views;

        for (auto kv : originatingViews) {
            views.insert(kv.first);
        }

        return views;
    }

    void setPoint(double x, double y, double z) {
        pt.x = x;
        pt.y = y;
        pt.z = z;
    }
};

class PointCloud {
private:
    std::vector<Point3DInMap> mReconstructionCloud;

public:
    PointCloud();

    void addPoint(const Point3DInMap& pt);

    void updatePoint(size_t i, double x, double y, double z);

    void mergePoints(PointCloud& pc, Matches& matches); // Merge an existing point cloud into this one

    Point3DInMap operator[](size_t i);
    std::vector<Point3DInMap>::iterator begin();
    std::vector<Point3DInMap>::iterator end();
    size_t size() const;
};

#endif //SFM_POINTCLOUD_H
