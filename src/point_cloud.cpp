//
// Created by Megan Finch on 29/12/2021.
//

#include <headers/constants.h>
#include <headers/point_cloud.h>
#include <vector>
#include <headers/matches.h>
#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/poisson.h>

#include <headers/vector_util.h>

typedef pcl::PointXYZ PointType;
typedef pcl::PointNormal PointTypeN;

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

void PointCloud::removePoints(std::vector<int>& indices_to_remove) {
    VectorUtilities::removeIndicesFromVector(mReconstructionCloud, indices_to_remove);
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

    // Stupid that this doesn't auto-update as you add points!
    pcl_cloud.width = pcl_cloud.size();
    pcl_cloud.height = 1;

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

void PointCloud::toVTKFile(const std::string& vtk_mesh_filename,
                           const std::string& pcd_normals_filename,
                           int psn_depth,
                           int psn_solverDivide,
                           int psn_isoDivide,
                           float psn_samplesPerNode,
                           float psn_scale,
                           int psn_confidence) {
    std::cout << "--- Converting point cloud to surface ---" << std::endl;
    // Convert cloud to PCL point cloud
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    for (const auto& point3D : *this) {
        // point constructor: pcl_point(x,y,z); - all std::uint8_t
        pcl::PointXYZ pcl_point(static_cast<float>(point3D.pt.x),
                                static_cast<float>(point3D.pt.y),
                                static_cast<float>(point3D.pt.z));

        cloud->points.emplace_back(pcl_point);
    }

    // Normal estimation
    pcl::NormalEstimation<PointType, pcl::Normal> normEst;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    // Create kdtree representation of cloud,
    // and pass it to the normal estimation object.
    pcl::search::KdTree<PointType>::Ptr tree (new
        pcl::search::KdTree<PointType>);
    tree->setInputCloud(cloud);
    normEst.setInputCloud(cloud);
    normEst.setSearchMethod(tree);
    // Estimate normal from 20 neighbour points
    normEst.setKSearch(20);
    normEst.compute(*normals);

    // Concatenate XYZ and normal fields
    pcl::PointCloud<PointTypeN>::Ptr cloud_with_normals (
            new pcl::PointCloud<PointTypeN>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

    pcl::search::KdTree<PointTypeN>::Ptr tree2 (new
        pcl::search::KdTree<PointTypeN>);
    tree2->setInputCloud(cloud_with_normals);

    // Set up Poisson reconstruction
    pcl::Poisson<PointTypeN> psn;
    pcl::PolygonMesh triangles;

    psn.setInputCloud(cloud_with_normals);
    psn.setSearchMethod(tree2);
    psn.setDepth(psn_depth);
    psn.setSolverDivide(psn_solverDivide);
    psn.setIsoDivide(psn_isoDivide);
    psn.setSamplesPerNode(psn_samplesPerNode);
    psn.setScale(psn_scale);
    psn.setConfidence(psn_confidence);
    psn.reconstruct(triangles);

    pcl::PCDWriter writer;
    pcl::PCLPointCloud2::Ptr cwn (new pcl::PCLPointCloud2());
    pcl::toPCLPointCloud2(*cloud_with_normals, *cwn);

    pcl::io::saveVTKFile(vtk_mesh_filename, triangles);
    writer.write(pcd_normals_filename, *cwn, Eigen::Vector4f::Zero(),
                 Eigen::Quaternionf::Identity(), false);
}
