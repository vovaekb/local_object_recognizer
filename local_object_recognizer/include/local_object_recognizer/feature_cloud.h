#ifndef FEATURE_CLOUD_H
#define FEATURE_CLOUD_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/pcl_macros.h>
#include <pcl/io/pcd_io.h>

using namespace std;

using PointType = pcl::PointXYZRGB;
using PointCloudTypePtr = pcl::PointCloud<PointType>::Ptr;
using PointCloudTypeConstPtr = pcl::PointCloud<PointType>::ConstPtr;
using NormalType = pcl::Normal;
using NormalCloudType = pcl::PointCloud<NormalType>::Ptr;
using SHOTDescriptorType = pcl::SHOT352;
using SHOTDescriptorCloudType = pcl::PointCloud<SHOTDescriptorType>::Ptr;

double computeCloudResolution(const PointCloudTypeConstPtr &cloud);

/*  FeatureCloud class */
class FeatureCloud
{
public:
    FeatureCloud();

    ~FeatureCloud();

    // Process the given cloud
    void setInputCloud(PointCloudTypePtr xyz, const string &cloud_type);
    // Load and process point cloud in the given PCD
    void loadInputCloud(const string &pcd_file, const string &cloud_type);

    void setPose(const Eigen::Matrix4f &&pose);

    void setModelId(const string &model_id);

    void setViewId(const int &view_id);

    PointCloudTypePtr getPointCloud() const;

    NormalCloudType getSurfaceNormals() const;

    PointCloudTypePtr getKeypoints() const;

    SHOTDescriptorCloudType getLocalFeatures() const;

    Eigen::Matrix4f getPose() const;

    std::string getModelId() const;

    int getViewId() const;

protected:
    void processInput();

    void computeSurfaceNormals();

    void extractKeypoints();

    void computeLocalFeatures();

private:
    // Point cloud data
    PointCloudTypePtr xyz_;
    NormalCloudType normals_;
    PointCloudTypePtr keypoints_;
    SHOTDescriptorCloudType features_;
    Eigen::Matrix4f pose_;

    string cloud_type_;
    int cloud_number_;
    string model_id_;
    int view_id_;
    // Algorithms parameters
    float norm_est_k_;
    float descr_rad_;
};

#endif // FEATURE_CLOUD_H
