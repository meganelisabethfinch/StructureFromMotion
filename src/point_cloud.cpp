//
// Created by Megan Finch on 29/12/2021.
//

#include <headers/constants.h>
#include <headers/point_cloud.h>
#include <vector>
#include <headers/matches.h>
#include <iostream>

PointCloud::PointCloud() {}

Point3DInMap PointCloud::operator[](size_t i) {
    return map[i];
}

std::vector<Point3DInMap>::iterator PointCloud::begin() {
    return map.begin();
}

std::vector<Point3DInMap>::iterator PointCloud::end() {
    return map.end();
}

size_t PointCloud::size() const {
    return map.size();
}

void PointCloud::addPoint(const Point3DInMap& pt) {
    map.push_back(pt);
}

void PointCloud::mergePoints(PointCloud &pc, Matches& matches) {
    MatchMatrix mergeMatchMatrix;
    mergeMatchMatrix.resize(matches.size(), std::vector<Matching2>(matches.size()));

    size_t newPoints = 0;
    size_t mergedPoints = 0;

    for (Point3DInMap& newPoint : pc) {

        bool foundAnyMatchingExistingViews = false;
        bool foundMatching3DPoint = false;

        for (Point3DInMap& existingPoint : map) {
            if (cv::norm(existingPoint.pt - newPoint.pt) < MERGE_CLOUD_POINT_MIN_MATCH_DISTANCE) {
                // New point is very close to an existing 3D cloud point
                foundMatching3DPoint = true;

                // Look for common 2D features to confirm match
                for (const auto& newKV : newPoint.originatingViews) {
                    for (const auto& existingKV : existingPoint.originatingViews) {
                        bool foundMatchingFeature = false;

                        const ImageID queryImageIndex = newKV.first < existingKV.first ? newKV.first : existingKV.first;
                        const ImageID trainImageIndex = newKV.first < existingKV.first ? existingKV.first : newKV.first;
                        const int queryFeatureIndex = newKV.first < existingKV.first ? newKV.second : existingKV.second;
                        const int trainFeatureIndex = newKV.first < existingKV.first ? existingKV.second : newKV.second;

                        const Matching2& matching = matches.getMatchingBetween(newKV.first, existingKV.first);
                        for (const cv::DMatch& m : matching) {
                            if (m.queryIdx == queryFeatureIndex
                                and m.trainIdx == trainFeatureIndex
                                and m.distance < MERGE_CLOUD_FEATURE_MIN_MATCH_DISTANCE)
                            {
                                mergeMatchMatrix[queryImageIndex][trainImageIndex].push_back(m);

                                // Found a 2D feature which is an originating view/point for both
                                // the new and existing 3D points - merge.
                                foundMatchingFeature = true;
                                break;
                            }
                        }

                        if (foundMatchingFeature) {
                            // Add the new originating view and feature index
                            existingPoint.originatingViews[newKV.first] = newKV.second;

                            foundAnyMatchingExistingViews = true;
                        }
                    }
                }
            }

            if (foundAnyMatchingExistingViews) {
                mergedPoints++;
                break; // out of existing points loop

                // newPoint has been merged into existingPoint,
                // and should only correspond to one existing point
            }
        }

        if (not foundAnyMatchingExistingViews and not foundMatching3DPoint) {
            // This point did not match any existing cloud points
            // Add it as a new point
            map.push_back(newPoint);
            newPoints++;
        }
    }

    std::cout << "Adding: " << pc.size() << " (new: " << newPoints << ", merged: " << mergedPoints << ")" << std::endl;
}
