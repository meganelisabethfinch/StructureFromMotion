//
// Created by Megan Finch on 24/02/2022.
//

#ifndef SFM_ZHANG_BUNDLE_ADJUSTER_H
#define SFM_ZHANG_BUNDLE_ADJUSTER_H

#include <headers/map_util.h>
#include <headers/vector_util.h>
#include "bundle_adjuster.h"

class ZhangBundleAdjuster : public BundleAdjuster {
    void adjustBundle(Bundle& bundle) override
    {
        // TODO: implement Zhang RBA (2006).
        // Call to formatPoints to get tuples

        // Format all poses into 6D vectors
        std::map<ImageID, PoseVector> cameraPoses6d;
        for (auto i : bundle.registeredImages) {
            auto pv = bundle.cameraPoses.at(i).toPoseVector();
            cameraPoses6d[i] = pv;
        }

        // Format all scene points into 3D vectors
        std::vector<cv::Vec3d> points3d(bundle.pointCloud.size());
        for (auto i = 0; i < bundle.pointCloud.size(); i++) {
            const Point3DInMap& p = bundle.pointCloud[i];
            points3d[i] = cv::Vec3d(p.pt.x, p.pt.y, p.pt.z);
        }


    }

private:
    void formatPoints(PointCloud& pointCloud, size_t pointsDesired, size_t viewsDesired) {
        // Find all combinations of 6 points seen by 3 cameras

        std::vector<size_t> points(pointsDesired);
        std::set<ImageID> sharedViews;
        std::vector<std::vector<size_t>> result = createPointCameraTuples(
                pointsDesired, viewsDesired,
                pointCloud,
                points, sharedViews,
                0, 0);


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
     * @return returns a nested vector [[P1, P2, P3, ..., C1, C2, C3], [...]].
     *         The first |pointsDesired| elements of each subvector are PointIDs (size_t).
     *         The remaining |viewsDesired| are ImageIDs (also size_t).
     */
    std::vector<std::vector<size_t>> createPointCameraTuples(size_t pointsDesired, size_t viewsDesired,
                                                             PointCloud& pointCloud,
                                std::vector<size_t> points, const std::set<ImageID>& sharedViews,
                                size_t pointIdx, size_t startIdx)
    {
        std::vector<std::vector<size_t>> result;

        for (size_t i = startIdx; i < pointCloud.size() - pointsDesired + 1; i++) {
            points[pointIdx] = i; // Only need to record ID, not point itself
            auto views = MapUtilities::ExtractKeys(pointCloud[i].originatingViews);
            std::set<ImageID> intersect;

            if (pointIdx == 0) {
                // slightly different behaviour for first point
                // set sharedViews rather than intersecting
                intersect = views;
            } else {
                std::set_intersection(sharedViews.begin(), sharedViews.end(),
                                      views.begin(), views.end(),
                                      std::inserter(intersect, intersect.begin()));
            }

            if (intersect.size() < viewsDesired) {
                // Early exit if not enough views.
                // No reason to consider points [1,2,3,...] if [1,2,3] alone don't have enough shared views.
                continue;
            }

            if (pointIdx + 1 == pointsDesired) {
                // BASE CASE: Found enough points.

                // Convert views to vector and find all combinations
                std::vector<ImageID> views_vec(intersect.begin(), intersect.end());
                auto views_combinations = VectorUtilities::generateCombinations<ImageID>
                        (views_vec, views_vec.size(), viewsDesired);

                for (auto& combo : views_combinations) {
                    std::vector<size_t> tuple;
                    // Add points
                    tuple.insert(tuple.end(), points.begin(), points.end());
                    // Add views
                    tuple.insert(tuple.end(), combo.begin(), combo.end());
                    // Add to result
                    result.push_back(tuple);
                }
            } else {
                // RECURSIVE CASE: Need more points.
                auto sub = createPointCameraTuples(pointsDesired, viewsDesired, pointCloud,
                                        points, intersect,
                                        pointIdx + 1, i + 1);
                result.insert(result.end(), sub.begin(), sub.end());
            }
        }

        return result;
    }

};
#endif //SFM_ZHANG_BUNDLE_ADJUSTER_H
