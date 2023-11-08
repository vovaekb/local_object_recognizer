#include <ros/ros.h>

#include <math.h>

// Boost
#include <boost/format.hpp>

#include <Eigen/Core>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/pcl_macros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "local_object_recognizer/common.h"
#include "local_object_recognizer/console_parse.h"
#include "local_object_recognizer/feature_cloud.h"
#include "local_object_recognizer/recognizer.h"
#include "local_object_recognizer/persistence_utils.h"

ros::Publisher pub;
ros::Publisher recogn_pub;
ros::Publisher model_pub;

using namespace std;

bool perform_verification(true);
bool use_hough(true);
bool use_hv(true);
bool visualize(false);

bool perform_ransac(false);
// Algorithm params
float descr_rad(0.03f);
double rf_radius(0.01);
double cg_size(0.04);
double cg_threshold(2.0);

int sac_max_iters(1000);
float sac_inlier_thresh(0.005);
float hv_inlier_thresh(0.06);

// Training scene and models paths
string scene_pcd_file;
string scene_name;
string model_name;
string models_dir;
string model_descr_dir;
string base_descr_dir("cloud_descr");
string gt_files_dir;

string gt_file_path;

const int templates_number(5);

const int icp_max_iter(30);
const float icp_corr_distance(0.01);
const float icp_fitness_eps(0.0002);
const float hv_clutter_reg(5.0f);
const float hv_rad_clutter(0.03f);
const float hv_regularizer(3.f);

// vector<string> templates_names;
Recognizer recognizer;

void runRecognizer()
{
    // Load scene cloud
    PointCloudPtr scene_cloud(new PointCloud());

    //
    // Load scene
    //
    cout << "\n\n ---------------- Loading scene ---------------------- \n\n";
    cout << "\n\nScene: " << scene_pcd_file << "\n";
    pcl::io::loadPCDFile(scene_pcd_file, *scene_cloud);

    FeatureCloud target_cloud;
    target_cloud.setInputCloud(scene_cloud, "scene");

    recognizer.setTrainingDir(models_dir);

    recognizer.setTargetCloud(target_cloud);

    recognizer.recognize();

    std::list<Recognizer::ObjectHypothesis, Eigen::aligned_allocator<Recognizer::ObjectHypothesis>> models = recognizer.getModels();

    // Visualize the found templates
    if (models.size())
    {
        // Calculate pose estimation error

        cout << "Calculate pose estimation error\n";

        if (!boost::filesystem::exists(gt_file_path))
        {
            ROS_ERROR("Ground truth file %s doesn't exist", gt_file_path.c_str());
            exit(-1);
        }

        for (auto &&model : models)
        {
            string model_id = model.model_template.getModelId();

            cout << "Model " << model_id;

            Eigen::Matrix4f gt_pose;

            if (PersistenceUtils::readGroundTruthFromFile(gt_file_path, model_id, gt_pose))
            {
                cout << " is present in the scene\n";

                // Transform the view pose to model RF
                Eigen::Matrix4f view_pose = model.transformation;

                Eigen::Matrix4f hom_matrix_pose = model.model_template.getPose();

                Eigen::Matrix4f model_pose = hom_matrix_pose.inverse() * view_pose;

                Eigen::Vector3f model_trans = model_pose.block<3, 1>(0, 3);
                Eigen::Matrix3f model_rot = model_pose.block<3, 3>(0, 0);

                Eigen::Vector3f gt_trans = gt_pose.block<3, 1>(0, 3);
                Eigen::Matrix3f gt_rot = gt_pose.block<3, 3>(0, 0);

                // norm-2: sqrt(x1^2 + x2^2 + x3^2)
                Eigen::Vector3f gt_est_trans_diff = gt_trans - model_trans;

                // Translation error
                float trans_err = static_cast<float>(gt_est_trans_diff.norm());

                cout << "Translation error: " << trans_err << "\n\n";

                //
                // Rotation error
                //
                // Rotation for X axis
                Eigen::Vector3f model_rot_x = model_rot.block<3, 1>(0, 0);

                Eigen::Vector3f gt_rot_x = gt_rot.block<3, 1>(0, 0);

                double rot_x_error = acos(static_cast<double>(gt_rot_x.dot(model_rot_x)));

                double rot_x_error_angle = rot_x_error * 180 / M_PI;

                printf("Rotation X error: %0.8f\n", rot_x_error);

                printf("Rotation X error angle: %0.8f\n", rot_x_error_angle);

                // Rotation for Y axis
                Eigen::Vector3f model_rot_y = model_rot.block<3, 1>(0, 1);

                Eigen::Vector3f gt_rot_y = gt_rot.block<3, 1>(0, 1);

                double rot_y_error = acos(static_cast<double>(gt_rot_y.dot(model_rot_y)));

                double rot_y_error_angle = rot_y_error * 180 / M_PI;

                printf("Rotation Y error: %0.8f\n", rot_y_error);

                printf("Rotation Y error angle: %0.6f\n", rot_y_error_angle);

                // Rotation for Z axis
                Eigen::Vector3f model_rot_z = model_rot.block<3, 1>(0, 2);

                Eigen::Vector3f gt_rot_z = gt_rot.block<3, 1>(0, 2);

                double rot_z_error = acos(static_cast<double>(gt_rot_z.dot(model_rot_z)));

                double rot_z_error_angle = rot_z_error * 180 / M_PI;

                printf("Rotation Z error: %0.8f\n", rot_z_error);

                printf("Rotation Z error angle: %0.6f\n", rot_z_error_angle);

                double avg_rot_error = (rot_x_error + rot_y_error + rot_z_error) / 3;
                double avg_rot_error_angle = avg_rot_error * 180 / M_PI;

                printf("Avarage rotation error: %0.8f\n", avg_rot_error);

                printf("Avarage rotation error angle: %0.8f\n", avg_rot_error_angle);
            }
            else
            {
                cout << " is absent in the scene\n";
            }
        }

        int positives_n = 0;
        int negatives_n = 0;
        int tp_n = 0;
        int fp_n = 0;
        int tn_n = 0;
        int fn_n = 0;

        vector<string> model_names = recognizer.getTrainingModelNames();
        for (int i = 0; i < model_names.size(); i++)
        {
            string model_name = model_names[i];
            bool is_present = PersistenceUtils::modelPresents(gt_file_path, model_name);
            bool is_found = false;

            for (auto &&model : models)
            {
                if (model.model_template.getModelId() == model_name)
                    is_found = true;
            }

            cout << "Model " << model_name << "\n";
            cout << "\t- is present: " << is_present << "\n";
            cout << "\t- is found: " << is_found << "\n\n";

            if (is_present)
            {
                if (is_found)
                {
                    tp_n++;
                    positives_n++;
                }
                else
                {
                    fn_n++;
                    negatives_n++;
                }
            }
            else
            {
                if (is_found)
                {
                    fp_n++;
                    positives_n++;
                }
                else
                {
                    tn_n++;
                    negatives_n++;
                }
            }
        }

        cout << "Positives: " << positives_n << "\n";
        cout << "Negatives: " << negatives_n << "\n";
        cout << "tp: " << tp_n << ", fp: " << fp_n << "\n";
        cout << "fn: " << fn_n << ", tn: " << tn_n << "\n";

        if (visualize)
        {
            cout << "------------ Visualization ---------------\n";
            pcl::visualization::PCLVisualizer viewer("Local recognition pipeline");

            int v1(0);
            pcl::visualization::PointCloudColorHandlerRGBField<PointType> scene_color_handler(scene_cloud);
            viewer.addPointCloud(scene_cloud, scene_color_handler, "scene_cloud");
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "scene_cloud");
            viewer.addCoordinateSystem(1.0);

            for (auto &&model : models)
            {
                PointCloudPtr model_cloud = model.model_template.getPointCloud();
                Eigen::Matrix4f transform = model.transformation; // 6DoF pose

                PointCloudPtr aligned_model(new PointCloud);
                pcl::transformPointCloud(*model_cloud, *aligned_model, transform);

                std::stringstream ss_cloud;
                ss_cloud << "model " << model.model_template.getModelId();

                pcl::visualization::PointCloudColorHandlerCustom<PointType> aligned_model_color_handler(aligned_model, 255, 0, 0);
                viewer.addPointCloud(aligned_model, aligned_model_color_handler, ss_cloud.str());
            }

            while (!viewer.wasStopped())
            {
                viewer.spinOnce();
            }
        }
    }
}

void getModelsFromDir(boost::filesystem::path &dir)
{
    //    cout << "[getModelsFromDir]\n";
    //    cout << "Reading dir: " << dir.string() << "\n";

    std::string model_id;
    if (dir != models_dir)
    {
        model_id = (dir.filename()).string();
        recognizer.addModelId(model_id);
    }

    boost::filesystem::directory_iterator end_itr;
    for (boost::filesystem::directory_iterator iter(dir); iter != end_itr; ++iter)
    {
        if (boost::filesystem::is_directory(*iter))
        {
            boost::filesystem::path curr_path = iter->path();

            getModelsFromDir(curr_path);
        }
        else
        {
            string file = (iter->path().filename()).string();
            string pcd_path = (iter->path()).string();

            int start_template_index = 1;
            int end_template_index = start_template_index + templates_number;

            vector<string> strs;

            boost::split(strs, file, boost::is_any_of("."));
            int view_id = atoi(strs[0].c_str());

            if (boost::filesystem::extension(iter->path()) == ".pcd" && file.substr(0, 5) != "descr" && view_id < end_template_index)
            {
                cout << "Loading model view " << file << "\n";

                stringstream ss;
                ss << dir.string() << "/pose_" << view_id << ".txt";

                string pose_file = ss.str();

                Eigen::Matrix4f pose;

                PersistenceUtils::readMatrixFromFile(pose_file, pose);

                FeatureCloud view_cloud;
                view_cloud.setViewId(view_id);
                view_cloud.setModelId(model_id);
                view_cloud.setPose(std::move(pose));
                view_cloud.loadInputCloud(pcd_path.c_str(), "model");
                recognizer.addTemplateCloud(view_cloud);
            }
        }
    }
}

void recognize()
{
    cout << "*** Run recognition ***\n";

    // Check existence of basic directory for descriptors
    if (!boost::filesystem::exists(base_descr_dir))
        boost::filesystem::create_directory(base_descr_dir);

    std::stringstream ss;
    ss << base_descr_dir << "/" << model_name;
    model_descr_dir = ss.str();

    if (!boost::filesystem::exists(model_descr_dir))
        boost::filesystem::create_directories(model_descr_dir);

    cout << "Load models\n";
    boost::filesystem::path models_path = models_dir;

    getModelsFromDir(models_path);

    recognizer.printModels();

    runRecognizer();
}

int main(int argc, char **argv)
{
    cout << "local_object_recognizer init [updated]\n";

    ros::init(argc, argv, "recognizer_node");

    ros::NodeHandle nh;

    ConsoleUtils::parseCommandLine();

    cout << "models_dir: " << models_dir << "\n";
    cout << "gt_files_dir: " << gt_files_dir << "\n";

    cout << "scene_pcd: " << scene_pcd_file << "\n";

    cout << "model: " << model_name << "\n";
    cout << "visualize: " << visualize << "\n";
    cout << "ransac: " << perform_ransac << "\n";

    cout << "rf_radius: " << rf_radius << "\n";
    cout << "cg_size: " << cg_size << "\n";
    cout << "cg_thresh: " << cg_threshold << "\n";
    cout << "sac_max_iters: " << sac_max_iters << "\n";
    cout << "sac_inlier_thresh: " << sac_inlier_thresh << "\n";
    cout << "hv_inlier_thresh: " << hv_inlier_thresh << "\n";

    std::size_t pos = scene_pcd_file.find_last_of("/") + 1;
    scene_name = scene_pcd_file.substr(pos);

    scene_name = scene_name.substr(0, scene_name.find(".pcd"));

    cout << "scene_name: " << scene_name << "\n";

    string gt_file = scene_name + ".txt";

    stringstream gt_path_ss;
    gt_path_ss << gt_files_dir << "/" << gt_file;

    gt_file_path = gt_path_ss.str();

    cout << "gt file path: " << gt_file_path << "\n";

    pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);
    recogn_pub = nh.advertise<sensor_msgs::PointCloud2>("recogn", 1);
    model_pub = nh.advertise<sensor_msgs::PointCloud2>("off_scene_model", 1);

    recognize();

    ros::spin();
}
