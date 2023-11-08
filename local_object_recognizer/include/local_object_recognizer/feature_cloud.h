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

#include "typedefs.h"

using namespace std;

double computeCloudResolution(const PointCloudConstPtr &cloud);

/*  FeatureCloud class */
class FeatureCloud
{
public:
    FeatureCloud();

    ~FeatureCloud();

    // Process the given cloud
    void setInputCloud(PointCloudPtr xyz, const string &cloud_type);
    // Load and process point cloud in the given PCD
    void loadInputCloud(const string &pcd_file, const string &cloud_type);

    void setPose(const Eigen::Matrix4f &&pose);

    void setModelId(const string &model_id);

    void setViewId(const int &view_id);

    PointCloudPtr getPointCloud() const;

    SurfaceNormalsPtr getSurfaceNormals() const;

    PointCloudPtr getKeypoints() const;

    SHOTDescriptorCloudPtr getLocalFeatures() const;

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
    PointCloudPtr xyz_;
    SurfaceNormalsPtr normals_;
    PointCloudPtr keypoints_;
    SHOTDescriptorCloudPtr features_;
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
