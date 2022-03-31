//
// Created by Megan Finch on 31/03/2022.
//

#ifndef SFM_FILTER_H
#define SFM_FILTER_H

class Filter {
public:
    virtual void filterOutliers(PointCloud& pointCloud) = 0;

    static cv::Ptr<Filter> create(const FilterType& filterType);
};

#endif //SFM_FILTER_H
