#include <ros/ros.h>

#include <iostream>
#include <boost/format.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType>::Ptr PointCloudTypePtr;
typedef pcl::PointCloud<PointType>::ConstPtr PointCloudTypeConstPtr;

string samples_path;
string output_pcd_dir;
string output_path(".");

// Static parameters
int clouds_number (6);
float voxel_leaf_size_ (0.001);

// Containers for objects
vector<PointCloudTypePtr> clouds;

vector<int> mapping;

void process_cloud(PointCloudTypePtr &cloud, int index)
{
    // Scale cloud, ...
    cout << "Scale cloud\n";
    Eigen::Matrix4f cloud_transform = Eigen::Matrix4f::Identity();

    cloud_transform(0,0) = 0.001;
    cloud_transform(1,1) = 0.001;
    cloud_transform(2,2) = 0.001;

    pcl::transformPointCloud(*cloud, *cloud, cloud_transform);

    // ... downsample, ...
    cout << "Downsample cloud\n";
    pcl::VoxelGrid<PointType> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);

    PointCloudTypePtr temp_cloud (new pcl::PointCloud<PointType> ());
    voxel_grid.filter(*temp_cloud);
    cloud = temp_cloud;

    // ... and remove NaNs
    cout << "Remove NaN\n";
    pcl::removeNaNFromPointCloud(*cloud, *cloud, mapping);

    // Save point cloud
    cout << "Save cloud\n";
    string output_pcd = output_pcd_dir + "/" + boost::to_string(index) + ".pcd";
    pcl::io::savePCDFileASCII(output_pcd, *cloud);

    cout << "Point cloud was saved as " + output_pcd + "\n";
}

void process()
{
    if(!boost::filesystem::exists(output_pcd_dir)) boost::filesystem::create_directory(output_pcd_dir);

    // Load point clouds
    for(int i = 1; i <= clouds_number; i++)
    {
        string pcd_path = samples_path + "/" + boost::to_string(i) + ".pcd";

        if(boost::filesystem::exists(pcd_path))
        {
            PointCloudTypePtr cloud (new pcl::PointCloud<PointType> ());
            if(pcl::io::loadPCDFile(pcd_path, *cloud) != 0)
            {
                return;
            }

            cout << "Point cloud " << boost::to_string(i) << ".pcd has " << cloud->points.size() << " points\n";

            process_cloud(cloud, i);
        }
    }
}

void showHelp()
{
    cout << "\t\t ** process_cloud package **\n";
    cout << "Usage: rosrun local_object_recognizer view_preprocess _samples_path:=<samples_path> [options]\n";
    cout << "* where _samples_path - path to the directory of clouds to process\n";
    cout << "* where options are:\n";
    cout << "_clouds_n:=clouds_n - number of files to process\n";
}

void parseCommandLine()
{
    ros::NodeHandle private_node_handle_("~");

    bool show_help(false);
    private_node_handle_.getParam("h", show_help);

    if(show_help)
    {
        showHelp();
        exit(0);
    }

    private_node_handle_.getParam("samples_path", samples_path);

    if(samples_path == "")
    {
        cout << "Samples directory missing!\n";
        showHelp();
        exit(-1);
    }

    private_node_handle_.getParam("clouds_n", clouds_number);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "view_preprocess");

    parseCommandLine();

    std::stringstream output_ss;
    output_ss << output_path << "/" << samples_path << "_output";
    output_pcd_dir = output_ss.str();

    cout << "Samples dir: " << samples_path << "\n";
    cout << "Output samples dir: " << output_pcd_dir << "\n";
    cout << "Clouds number: " << clouds_number << "\n";
    process();
    return 0;
}

