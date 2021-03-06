#include <iostream>
#include <boost/format.hpp>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/pcd_io.h>
#include <vtkPolyData.h>
#include <vtkTriangle.h>
#include <vtkSmartPointer.h>
#include <vtkCellArray.h>
#include <vtkPLYReader.h>
#include <vtkPolyDataMapper.h>
#include <vtkTransform.h>
#include <vtkTransformFilter.h>
#include <pcl/apps/render_views_tesselated_sphere.h>
#include "local_object_recognizer/persistence_utils.h"

using namespace std;

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType>::Ptr PointCloudTypePtr;

int
main(int argc, char** argv)
{
    // Perform uniform sampling
    string ply_file(argv[1]);

    size_t pos = ply_file.find_last_of(".");

    string model_name = ply_file.substr(0, pos);

    std::cout << "model name: " << model_name << "\n";

    string samples_dir = model_name;
    boost::filesystem::create_directory(samples_dir);

    // Load the PLY model from a file.
    vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
    reader->SetFileName(argv[1]);
    reader->Update();

    // VTK is not exactly straightforward...
    vtkSmartPointer < vtkPolyDataMapper > mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(reader->GetOutputPort());
    mapper->Update();

    vtkSmartPointer<vtkPolyData> object = mapper->GetInput();

    // Virtual scanner object.
    pcl::apps::RenderViewsTesselatedSphere render_views;
    render_views.addModelFromPolyData(object);
    // Pixel width of the rendering window, it directly affects the snapshot file size.
    render_views.setResolution(150);
    // Horizontal FoV of the virtual camera.
    render_views.setViewAngle(57.0f);
    // If true, the resulting clouds of the snapshots will be organized.
    render_views.setGenOrganized(true);
    // How much to subdivide the icosahedron. Increasing this will result in a lot more snapshots.
    render_views.setTesselationLevel(1);
    // If true, the camera will be placed at the vertices of the triangles. If false, at the centers.
    // This will affect the number of snapshots produced (if true, less will be made).
    // True: 42 for level 1, 162 for level 2, 642 for level 3...
    // False: 80 for level 1, 320 for level 2, 1280 for level 3...
    render_views.setUseVertices(true);
    // If true, the entropies (the amount of occlusions) will be computed for each snapshot (optional).
    render_views.setComputeEntropies(true);

    render_views.generateViews();

    // Object for storing the rendered views.
    std::vector<PointCloudTypePtr> views;
    // Object for storing the poses, as 4x4 transformation matrices.
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > poses;
    // Object for storing the entropies (optional).
    std::vector<float> entropies;
    render_views.getViews(views);
    render_views.getPoses(poses);
    render_views.getEntropies(entropies);

    for(size_t j = 0; j < poses.size(); j++) //
    {
        cout << "Instance " << (j + 1) << ":\n";

        stringstream pose_ss;
        pose_ss << samples_dir << "/pose_" << j << ".txt";

        PersistenceUtils::writeMatrixToFile(pose_ss.str(), poses[j]);

        Eigen::Matrix3f rotation = poses[j].block<3, 3>(0, 0);
        Eigen::Vector3f translation = poses[j].block<3, 1>(0, 3);
        printf("\t\t    | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
        printf("\t\tR = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
        printf("\t\t    | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
        cout << "\n";
        printf("\t\tt = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));
    }

    for(size_t i = 0; i < poses.size(); i++)
    {
        cout << "Cloud " << i << " has " << views[i]->points.size() << " points\n";
        stringstream ss_cloud;
        ss_cloud << samples_dir << "/" << i << ".pcd";
        cout << ss_cloud.str() << "\n";
        PointCloudTypePtr cloud_sample (new pcl::PointCloud<PointType> ());
        *cloud_sample = *views[i];
        pcl::io::savePCDFileASCII(ss_cloud.str(), *cloud_sample);

        cout << ss_cloud.str() << " was saved\n";
    }
}

