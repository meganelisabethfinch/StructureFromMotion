//
// Created by Megan Finch on 29/12/2021.
//

#include <headers/constants.h>
#include <headers/point_cloud.h>
#include <vector>
#include <headers/matches.h>
#include <iostream>
#include <fstream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <headers/vector_util.h>

PointCloud::PointCloud() {}

Point3DInMap PointCloud::operator[](size_t i) {
    return mReconstructionCloud[i];
}

std::vector<Point3DInMap>::iterator PointCloud::begin() {
    return mReconstructionCloud.begin();
}

std::vector<Point3DInMap>::iterator PointCloud::end() {
    return mReconstructionCloud.end();
}

size_t PointCloud::size() const {
    return mReconstructionCloud.size();
}

void PointCloud::addPoint(const Point3DInMap& pt) {
    mReconstructionCloud.push_back(pt);
}

void PointCloud::updatePoint(size_t i, double x, double y, double z) {
    mReconstructionCloud[i].pt.x = x;
    mReconstructionCloud[i].pt.y = y;
    mReconstructionCloud[i].pt.z = z;
}

void PointCloud::mergePoints(PointCloud &pc, Matches& matches, double mergePointDistance, double mergeFeatureDistance) {
    MatchMatrix mergeMatchMatrix;
    mergeMatchMatrix.resize(matches.size(), std::vector<Matching2>(matches.size()));

    size_t newPoints = 0;
    size_t mergedPoints = 0;

    for (Point3DInMap& newPoint : pc) {
        bool foundAnyMatchingExistingViews = false;
        bool foundMatching3DPoint = false;

        for (Point3DInMap& existingPoint : mReconstructionCloud) {
            if (cv::norm(existingPoint.pt - newPoint.pt) < mergePointDistance) {
                // New point is very close to an existing 3D cloud point
                foundMatching3DPoint = true;

                // Look for common 2D features to confirm match
                for (const auto& newKV : newPoint.originatingViews) {
                    for (const auto& existingKV : existingPoint.originatingViews) {
                        bool foundMatchingFeature = false;

                        const int queryFeatureIndex = newKV.first < existingKV.first ? newKV.second : existingKV.second;
                        const int trainFeatureIndex = newKV.first < existingKV.first ? existingKV.second : newKV.second;

                        if (newKV.first == existingKV.first) { continue; }

                        auto ip = ImagePair(newKV.first, existingKV.first);
                        const Matching2& matching = matches.get(ip);
                        for (const cv::DMatch& m : matching) {
                            if (m.queryIdx == queryFeatureIndex
                                and m.trainIdx == trainFeatureIndex
                                and m.distance < mergeFeatureDistance)
                            {
                                mergeMatchMatrix[ip.left][ip.right].push_back(m);

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
            mReconstructionCloud.push_back(newPoint);
            newPoints++;
        }
    }

    if (DEFAULT_DEBUG >= DebugLevel::VERBOSE) {
        std::cout << "Adding: " << pc.size() << " (new: " << newPoints << ", merged: " << mergedPoints << ")"
                  << std::endl;
    }
}

void PointCloud::pruneStatisticalOutliers(int k, double stddev_mult) {
    std::cout << "--- Remove Statistical Outliers ---" << std::endl;
    // Convert cloud to PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& point3D : *this) {
        // point constructor: pcl_point(x,y,z); - all std::uint8_t
        pcl::PointXYZ pcl_point(static_cast<float>(point3D.pt.x),
                                static_cast<float>(point3D.pt.y),
                                static_cast<float>(point3D.pt.z));

        cloud_in->points.emplace_back(pcl_point);
    }

    // Set up filter
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sorfilter(true);
    sorfilter.setInputCloud(cloud_in);
    sorfilter.setMeanK(8);
    sorfilter.setStddevMulThresh(stddev_mult);

    // Apply filter and extract outliers
    pcl::PointCloud<pcl::PointXYZ> cloud_out;
    sorfilter.filter(cloud_out);
    pcl::IndicesConstPtr rm = sorfilter.getRemovedIndices();

    // Convert rm to std::vector<int>
    std::vector<int> outlier_indices;
    for (int i : *rm) {
        outlier_indices.push_back(i);
    }

    // Prune this point cloud
    VectorUtilities::removeIndicesFromVector(mReconstructionCloud, outlier_indices);
    std::cout << "SOR Result: point cloud reduced from " << cloud_in->size() << " points -> "
        << mReconstructionCloud.size() << " points" << std::endl;
}

pcl::PointCloud<pcl::PointXYZRGB> PointCloud::toPCLPointCloud(const std::vector<Features>& features,
                                                              const std::vector<Image>& images) {
    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;

    for (const auto& point3D : *this) {
        auto anyOriginatingView = point3D.originatingViews.begin();
        const ImageID viewIdx = anyOriginatingView->first;
        const int keypointIdx = anyOriginatingView->second;
        cv::Point2d point2D = features.at(viewIdx).getPoint(keypointIdx);
        cv::Vec3b pointColour = images.at(viewIdx).getColourAt(point2D);


        // point constructor: pcl_point(x,y,z,r,g,b); - all std::uint8_t
        pcl::PointXYZRGB pcl_point(static_cast<float>(point3D.pt.x),
                                   static_cast<float>(point3D.pt.y),
                                   static_cast<float>(point3D.pt.z),
                                   pointColour[0], pointColour[1], pointColour[2]);

        pcl_cloud.points.emplace_back(pcl_point);
    }

    return pcl_cloud;
}

void PointCloud::toPlyFile(const std::string& filename,
                           const std::vector<Features>& features,
                           const std::vector<Image>& images) {
    std::cout << "Converting point cloud to .PLY file." << std::endl;

    std::ofstream file(filename);
    file << "ply" << std::endl;
    file << "format ascii 1.0" << std::endl;
    file << "element vertex " << this->size() << std::endl;
    file << "property float x" << std::endl;
    file << "property float y" << std::endl;
    file << "property float z" << std::endl;
    file << "property uchar red" << std::endl;
    file << "property uchar green" << std::endl;
    file << "property uchar blue" << std::endl;
    file << "end_header" << std::endl;

    for (const auto& point3D : *this) {
        auto anyOriginatingView = point3D.originatingViews.begin();
        const ImageID viewIdx = anyOriginatingView->first;
        const int keypointIdx = anyOriginatingView->second;
        cv::Point2d point2D = features.at(viewIdx).getPoint(keypointIdx);
        cv::Vec3b pointColour = images.at(viewIdx).getColourAt(point2D);

        file << static_cast<float>(point3D.pt.x) << " ";
        file << static_cast<float>(point3D.pt.y) << " ";
        file << static_cast<float>(point3D.pt.z) << " ";
        file << (int)pointColour(2) << " ";
        file << (int)pointColour(1) << " ";
        file << (int)pointColour(0) << std::endl;
    }

    file.close();
}

void PointCloud::toPCDFile(const std::string& filename,
               const std::vector<Features>& features,
               const std::vector<Image>& images)
{
    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud = this->toPCLPointCloud(features, images);
    pcl::io::savePCDFileASCII(filename, pcl_cloud);
}

