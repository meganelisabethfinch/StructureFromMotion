//
// Created by Megan Finch on 23/12/2021.
//

#ifndef SFM_POINTCLOUD_H
#define SFM_POINTCLOUD_H

#include <map>
#include <set>
#include "types.h"
#include "matches.h"
#include "constants.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

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

    pcl::PointCloud<pcl::PointXYZRGB> toPCLPointCloud(const std::vector<Features>& features,
                                                      const std::vector<Image>& images);

public:
    PointCloud();

    void addPoint(const Point3DInMap& pt);

    void updatePoint(size_t i, double x, double y, double z);

    void removePoints(std::vector<int>& indices_to_remove);

    void mergePoints(PointCloud& pc,
                     Matches& matches,
                     double mergePointDistance = MERGE_CLOUD_POINT_MIN_MATCH_DISTANCE,
                     double mergeFeatureDistance = MERGE_CLOUD_FEATURE_MIN_MATCH_DISTANCE);
    // Merge an existing point cloud into this one

    void toPlyFile(const std::string& filename,
                   const std::vector<Features>& features,
                   const std::vector<Image>& images);

    void toPCDFile(const std::string& filename,
                   const std::vector<Features>& features,
                   const std::vector<Image>& images);

    void toVTKFile(const std::string& vtk_mesh_filename,
                   const std::string& pcd_normals_filename,
                   int psn_depth = 8,
                   int psn_solverDivide = 8,
                   int psn_isoDivide = 8,
                   float psn_samplesPerNode = 3,
                   float psn_scale = 1.25,
                   int psn_confidence = 1);

    Point3DInMap operator[](size_t i);
    std::vector<Point3DInMap>::iterator begin();
    std::vector<Point3DInMap>::iterator end();
    size_t size() const;
};

#endif //SFM_POINTCLOUD_H
