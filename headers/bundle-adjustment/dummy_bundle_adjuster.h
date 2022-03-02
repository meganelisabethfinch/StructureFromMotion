//
// Created by Megan Finch on 24/02/2022.
//

#ifndef SFM_DUMMY_BUNDLE_ADJUSTER_H
#define SFM_DUMMY_BUNDLE_ADJUSTER_H

#include "bundle_adjuster.h"

// Because sometimes less is more
class DummyBundleAdjuster : public BundleAdjuster {
    void adjustBundle(Bundle& bundle) override
    {
        // Do literally nothing.
    };
};

#endif //SFM_DUMMY_BUNDLE_ADJUSTER_H
