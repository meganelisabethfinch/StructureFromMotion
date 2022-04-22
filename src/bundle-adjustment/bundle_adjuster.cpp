//
// Created by Megan Finch on 24/02/2022.
//

#include <headers/bundle-adjustment/bundle_adjuster.h>
#include <headers/types.h>

#include <headers/bundle-adjustment/dummy_bundle_adjuster.h>
#include <headers/bundle-adjustment/basic_bundle_adjuster.h>
#include <headers/bundle-adjustment/zhang_bundle_adjuster.h>

cv::Ptr<BundleAdjuster> BundleAdjuster::create(const BundleAdjusterType& type, const LossType& loss) {
    switch (type) {
        case BundleAdjusterType::OFF:
            return cv::makePtr<DummyBundleAdjuster>();
        case BundleAdjusterType::BASIC:
            return cv::makePtr<BasicBundleAdjuster>(loss);
        case BundleAdjusterType::ZHANG:
            return cv::makePtr<ZhangBundleAdjuster>();
    }
}
