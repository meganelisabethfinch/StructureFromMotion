//
// Created by Megan Finch on 31/03/2022.
//

#ifndef SFM_STATISTICAL_OUTLIER_FILTER_H
#define SFM_STATISTICAL_OUTLIER_FILTER_H

class StatisticalOutlierFilter : public Filter {
private:
    int k;
    double stddev_mult;

public:
    void filterOutliers(PointCloud& pointCloud) override {
        std::cout << "--- Remove Statistical Outliers ---" << std::endl;
        // Convert cloud to PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& point3D : pointCloud) {
            // point constructor: pcl_point(x,y,z); - all std::uint8_t
            pcl::PointXYZ pcl_point(static_cast<float>(point3D.pt.x),
                                    static_cast<float>(point3D.pt.y),
                                    static_cast<float>(point3D.pt.z));

            cloud_in->points.emplace_back(pcl_point);
        }

        // Set up filter
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sorfilter(true);
        sorfilter.setInputCloud(cloud_in);
        sorfilter.setMeanK(k);
        sorfilter.setStddevMulThresh(stddev_mult);

        // Apply filter and extract outliers
        pcl::PointCloud<pcl::PointXYZ> cloud_out;
        sorfilter.filter(cloud_out);
        pcl::IndicesConstPtr rm = sorfilter.getRemovedIndices();

        // Convert rm to std::vector<int>
        std::vector<int> outlier_indices;
        for (int i : *rm) {
            outlier_indices.push_back(i);
        }

        // Prune this point cloud
        VectorUtilities::removeIndicesFromVector(mReconstructionCloud, outlier_indices);
        std::cout << "SOR Result: point cloud reduced from " << cloud_in->size() << " points -> "
                  << mReconstructionCloud.size() << " points" << std::endl;
    }
};

#endif //SFM_STATISTICAL_OUTLIER_FILTER_H
