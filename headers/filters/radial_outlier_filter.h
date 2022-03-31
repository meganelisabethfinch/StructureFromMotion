//
// Created by Megan Finch on 31/03/2022.
//

#ifndef SFM_RADIAL_OUTLIER_FILTER_H
#define SFM_RADIAL_OUTLIER_FILTER_H

class RadialOutlierFilter : public Filter {
private:
    double radiusSearch;
    int minNeighborsInRadius;

public:
    void filterOutliers(PointCloud& pointCloud) override {
        std::cout << "--- Remove Radial Outliers ---" << std::endl;
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
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> rorfilter(true);
        rorfilter.setInputCloud(cloud_in);
        rorfilter.setRadiusSearch(radiusSearch);
        rorfilter.setMinNeighborsInRadius(minNeighborsInRadius);
        rorfilter.setNegative(false);
        // When negative = true, we get points with < N neighbours in the search radius
        // When negative = false, we get points with >= N neighbours in the search radius

        // Apply filter and extract outliers
        pcl::PointCloud<pcl::PointXYZ> cloud_out;
        rorfilter.filter(cloud_out);
        pcl::IndicesConstPtr rm = rorfilter.getRemovedIndices();

        // Convert rm to std::vector<int>
        std::vector<int> outlier_indices;
        for (int i : *rm) {
            outlier_indices.push_back(i);
        }

        // Prune this point cloud
        VectorUtilities::removeIndicesFromVector(mReconstructionCloud, outlier_indices);
        std::cout << "ROR Result: point cloud reduced from " << cloud_in->size() << " points -> "
                  << mReconstructionCloud.size() << " points" << std::endl;
    }

    static cv::Ptr<RadialOutlierFilter> create(double radiusSearch, int minNeighborsInRadius);
};

#endif //SFM_RADIAL_OUTLIER_FILTER_H
