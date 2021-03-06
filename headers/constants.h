//
// Created by Megan Finch on 23/12/2021.
//

#ifndef SFM_CONSTANTS_H
#define SFM_CONSTANTS_H
#include "types.h"

const double LOWE_RATIO = 0.6;
const double FOCAL_LENGTH = 2500;

// Reject a 3d point if its re-projection error is above this threshold
const double REPROJECTION_ERROR_THRESHOLD = 3.0;

// RANSAC inlier threshold
const double RANSAC_THRESHOLD = 10.0;

// Reject a camera pose if the ratio of inliers is below this threshold
const double POSE_INLIERS_MINIMAL_RATIO = 0.5;

// Number of matches needed for an edge between two images
const int SCENE_GRAPH_EDGE_THRESHOLD = 25;

const double MERGE_CLOUD_POINT_MIN_MATCH_DISTANCE = 0.01;
const double MERGE_CLOUD_FEATURE_MIN_MATCH_DISTANCE = 20.0;

const int PNP_RANSAC_MAX_ITERATIONS = 100;
const double PNP_RANSAC_CONFIDENCE = 0.99;

const double FM_RANSAC_THRESHOLD = 3.0; // 3.0
const double FM_RANSAC_CONFIDENCE = 0.99; // 0.99

// Termination criteria for bundle adjustment
const double BA_ETA = 1e-2;

// Number of matching points needed to calculate matrices
const int POINTS_NEEDED_HOMOGRAPHY_MATRIX = 100;
const int POINTS_NEEDED_ESSENTIAL_MATRIX = 10;
const int POINTS_NEEDED_FUNDAMENTAL_MATRIX = 10;

// SOR - considers K nearest neighbours
const int SOR_K = 6;
const double SOR_STDDEV_MULT = 2.0;

// ROR - searches for k neighbours in radius
const double ROR_RADIUS = 0.1;
const int ROR_MIN_NEIGHBOURS = 5;

const DetectorType DEFAULT_DETECTOR = DetectorType::SIFT;
const MatcherType DEFAULT_MATCHER = MatcherType::BRUTEFORCE;
const TriangulatorType DEFAULT_TRIANGULATOR = TriangulatorType::LINEAR;
const BundleAdjusterType DEFAULT_BUNDLE_ADJUSTER = BundleAdjusterType::BASIC;
const LossType DEFAULT_LOSS_TYPE = LossType::NULL_LOSS;
const bool DEFAULT_USE_HOMOGRAPHY_ORDERING = true;

const DebugLevel DEFAULT_DEBUG = DebugLevel::SIMPLE;
const VisualDebugLevel DEFAULT_VISUAL_DEBUG = VisualDebugLevel::NONE;

#endif //SFM_CONSTANTS_H
