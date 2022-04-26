//
// Created by Megan Finch on 31/03/2022.
//

#ifndef SFM_RADIAL_OUTLIER_FILTER_H
#define SFM_RADIAL_OUTLIER_FILTER_H

#include <headers/point_cloud.h>
#include <headers/vector_util.h>

class RadialOutlierFilter : public Filter {
private:
    double _radiusSearch;
    int _minNeighborsInRadius;

public:
    explicit RadialOutlierFilter(double radiusSearch = 0.1, int minNeighborsInRadius = 5) :
        _radiusSearch(radiusSearch), _minNeighborsInRadius(minNeighborsInRadius)
    {};

    static cv::Ptr<RadialOutlierFilter> create(double radiusSearch, int minNeighborsInRadius) {
        return cv::makePtr<RadialOutlierFilter>(radiusSearch, minNeighborsInRadius);
    };

    void filterOutliers(PointCloud& pointCloud) override {
        std::cout << "--- Remove Radial Outliers ---" << std::endl;
        // Convert cloud to PCL point cloud
        // TODO: call pickle

        // Prune this point cloud
        /*
        pointCloud.removePoints(outlier_indices);
        std::cout << "ROR Result: point cloud reduced from " << cloud_in->size() << " points -> "
                  << pointCloud.size() << " points" << std::endl;
                  */
    }
};

#endif //SFM_RADIAL_OUTLIER_FILTER_H
