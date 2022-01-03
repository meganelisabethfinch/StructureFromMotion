//
// Created by Megan Finch on 23/12/2021.
//

#ifndef SFM_CONSTANTS_H
#define SFM_CONSTANTS_H
#include "types.h"

const double LOWE_RATIO = 0.6;
const double FOCAL_LENGTH = 2500;

// Reject a 3d point if its re-projection error is above this threshold
const double REPROJECTION_ERROR_THRESHOLD = 10.0;

// RANSAC inlier threshold
const double RANSAC_THRESHOLD = 10.0;

// Reject a camera pose if the ratio of inliers is below this threshold
const double POSE_INLIERS_MINIMAL_RATIO = 0.15;

// Number of matches needed for an edge between two images
const int SCENE_GRAPH_EDGE_THRESHOLD = 25;

const double MERGE_CLOUD_POINT_MIN_MATCH_DISTANCE = 0.01;
const double MERGE_CLOUD_FEATURE_MIN_MATCH_DISTANCE = 20.0;

// Number of matching points needed to calculate essential matrix
const int POINTS_NEEDED_ESSENTIAL_MATRIX = 7;

const DetectorType DEFAULT_DETECTOR = DetectorType::SIFT;
const MatcherType DEFAULT_MATCHER = MatcherType::FLANNBASED;
const DebugLevel DEFAULT_DEBUG = DebugLevel::BASIC;

#endif //SFM_CONSTANTS_H
