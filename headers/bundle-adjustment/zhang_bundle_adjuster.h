//
// Created by Megan Finch on 24/02/2022.
//

#ifndef SFM_ZHANG_BUNDLE_ADJUSTER_H
#define SFM_ZHANG_BUNDLE_ADJUSTER_H

#include <headers/map_util.h>
#include "bundle_adjuster.h"

class ZhangBundleAdjuster : public BundleAdjuster {
    void adjustBundle(Bundle& bundle) override
    {
        // TODO: implement Zhang RBA (2006).
    }

private:
    void FormatPoints(PointCloud& pointCloud) {
        // Find all combinations of 6 points seen by 3 cameras
        size_t pointsNeeded = 6;
        size_t sharedCameras = 3;
        std::vector<cv::Point3d> points(pointsNeeded);

    }

    /*
     * @param pointsDesired - the number of points in the tuple
     * @param viewsDesired - the number of views in the tuple.
     *                    These views must be shared between all points in the tuple.
     * @param pointCloud - point cloud from which points are drawn.
     * @param points - the tuple of points under construction. Terminate when points.size() == pointsDesired
     * @param sharedViews - accumulator of views shared between all points in points so far
     * @param pointIdx - indicates this call is to fill the pointIdx'th position in the tuple,
     *                      i.e. points[pointIdx]
     * @param startIdx - start searching the pointCloud from startIdx up to
     *                      pointCloud.size() - pointsSize + pointIdx
     */
    void createPointCameraTuples(size_t pointsDesired, size_t viewsDesired, PointCloud& pointCloud,
                                std::vector<cv::Point3d> points, const std::set<ImageID>& sharedViews,
                                size_t pointIdx, size_t startIdx)
    {
        for (size_t i = startIdx; i < pointCloud.size() - pointsDesired + 1; i++) {
            points[pointIdx] = pointCloud[i].pt;
            auto views = MapUtilities::ExtractKeys(pointCloud[i].originatingViews);
            std::set<ImageID> intersect;
            std::set_intersection(sharedViews.begin(), sharedViews.end(),
                                  views.begin(), views.end(),
                                  std::inserter(intersect, intersect.begin()));

            if (intersect.size() < viewsDesired) {
                // Early exit if not enough views.
                // No reason to consider points [1,2,3,...] if [1,2,3] alone don't have enough shared views.
                continue;
            }

            if (pointIdx + 1 == pointsDesired) {
                // BASE CASE: Found enough points.


            } else {
                // RECURSIVE CASE: Need more points.
                createPointCameraTuples(pointsDesired, viewsDesired, pointCloud,
                                        points, intersect,
                                        pointIdx + 1, i + 1);
            }

        }
    }

};
#endif //SFM_ZHANG_BUNDLE_ADJUSTER_H
