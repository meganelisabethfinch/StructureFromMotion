//
// Created by Megan Finch on 31/03/2022.
//

#ifndef SFM_STATISTICAL_OUTLIER_FILTER_H
#define SFM_STATISTICAL_OUTLIER_FILTER_H

class StatisticalOutlierFilter : public Filter {
private:
    int _k;
    double _stddev_mult;

public:
    explicit StatisticalOutlierFilter(int k = 8, double stddev_mult = 1.0) :
            _k(k), _stddev_mult(stddev_mult)
    {}

    static cv::Ptr<StatisticalOutlierFilter> create(int k = 8, double stddev_mult = 1.0) {
        return cv::makePtr<StatisticalOutlierFilter>(k, stddev_mult);
    }

    void filterOutliers(PointCloud& pointCloud) override {
        std::cout << "--- Remove Statistical Outliers ---" << std::endl;
        // TODO: call pickle

        // Prune this point cloud
        /*
        pointCloud.removePoints(outlier_indices);
        std::cout << "SOR Result: point cloud reduced from " << cloud_in->size() << " points -> "
                  << pointCloud.size() << " points" << std::endl;
                  */
    }
};

#endif //SFM_STATISTICAL_OUTLIER_FILTER_H
