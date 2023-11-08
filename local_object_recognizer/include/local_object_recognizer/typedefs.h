#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Type definitions for Point Cloud
using PointType = pcl::PointXYZRGB;
using PointTypeNoColor = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointType>;
using PointCloudNoColor = pcl::PointCloud<PointTypeNoColor>;
using PointCloudPtr = PointCloud::Ptr;
using PointCloudNoColorPtr = PointCloudNoColor::Ptr;
using PointCloudConstPtr = PointCloud::ConstPtr;

// Type definitions for Normals
using NormalType = pcl::Normal;
using SurfaceNormals = pcl::PointCloud<NormalType>;
using SurfaceNormalsPtr = SurfaceNormals::Ptr;

// Type definitions for SHOT descriptor
using SHOTDescriptorType = pcl::SHOT352;
using SHOTDescriptorCloud = pcl::PointCloud<SHOTDescriptorType>;
using SHOTDescriptorCloudPtr = SHOTDescriptorCloud::Ptr;

// Type definitions for Reference Frames
using RFType = pcl::ReferenceFrame;
using RFCloud = pcl::PointCloud<RFType>;
using RFCloudPtr = RFCloud::Ptr;