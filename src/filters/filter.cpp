//
// Created by Megan Finch on 31/03/2022.
//

#include <headers/filters/filter.h>
#include <headers/types.h>

#include <headers/filters/statistical_outlier_filter.h>
#include <headers/filters/radial_outlier_filter.h>

cv::Ptr<Filter> Filter::create(const FilterType& type) {
    switch (type) {
        case FilterType::STATISTICAL:
            return cv::makePtr<StatisticalOutlierFilter>();
        case FilterType::RADIAL:
            return cv::makePtr<RadialOutlierFilter>();
    }
}