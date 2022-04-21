//
// Created by Megan Finch on 13/02/2022.
//

#ifndef SFM_BUNDLE_ADJUSTER_H
#define SFM_BUNDLE_ADJUSTER_H

#include <ceres/loss_function.h>
#include "../bundle.h"
#include "../point_cloud.h"
#include "../pose.h"
#include "../camera.h"

class BundleAdjuster {
public:
    LossType lossType = LossType::NULL_LOSS;

    /*
    * Helper to create loss functions. Parameter a = 1.0 indicates they are in un-scaled format.
    */
    static ceres::LossFunction* createLossFunction(LossType lossType) {
        switch (lossType) {
            case LossType::NULL_LOSS:
                return nullptr;
            case LossType::HUBER:
                return new ceres::HuberLoss(1.0);
            case LossType::SOFTLONE:
                return new ceres::SoftLOneLoss(1.0);
            case LossType::CAUCHY:
                return new ceres::CauchyLoss(1.0);
        }
    }

    virtual void adjustBundle(Bundle& bundle) = 0;

    static cv::Ptr<BundleAdjuster> create(const BundleAdjusterType& type, const LossType& loss = LossType::NULL_LOSS);
};

#endif //SFM_BUNDLE_ADJUSTER_H
