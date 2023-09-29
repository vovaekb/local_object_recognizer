#ifndef RECOGNIZER_H
#define RECOGNIZER_H

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET

#include <ros/ros.h>

// Boost
#include <boost/format.hpp>

#include <Eigen/Core>

// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/pcl_macros.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/recognition/3rdparty/metslib/mets.hh>
#include <pcl/recognition/hv/hv_go.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

#include "local_object_recognizer/common.h"
#include "local_object_recognizer/feature_cloud.h"
#include "local_object_recognizer/persistence_utils.h"

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType>::Ptr PointCloudTypePtr;
typedef pcl::PointCloud<PointType>::ConstPtr PointCloudTypeConstPtr;
typedef pcl::Normal NormalType;
typedef pcl::SHOT352 SHOTDescriptorType;

using namespace std;

class Recognizer
{
public:
    // A struct for storing alignment results
    struct ObjectHypothesis
    {
        FeatureCloud model_template;
        float icp_score;
        Eigen::Matrix4f transformation;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };


    typedef pcl::PointXYZRGB PointType;
    typedef pcl::Normal NormalType;
    typedef pcl::ReferenceFrame RFType;
    typedef pcl::SHOT352 SHOTDescriptorType;
    Recognizer();

    ~Recognizer ();

    void setTrainingDir(const string &training_dir);

    void setTargetCloud(FeatureCloud &target_cloud);

    void setRFRadius(const float &rf_radius);

    void setCGSize(const float &cg_s);

    void setCGThresh(const float &cg_th);

    void setHVInlierThresh(const float &hv_thresh);

    vector<ObjectHypothesis, Eigen::aligned_allocator<ObjectHypothesis> > getModels();

    vector<string> getTrainingModelNames();

    void addTemplateCloud(FeatureCloud &template_cloud);

    void addModelId(const string & model_name);

    void printModels();

    // Matching
    void match ();

    // CG for one template
    void group_template_correspondences (FeatureCloud &template_cloud, const int index);

    // CG over all templates
    void group_correspondences ();

    // HV
    void hypotheses_verification ();

    // ICP alignment over all templates
    void alignAll ();

    // Find the best object hypotheses and their 6DOF poses
    void recognize();

private:
    vector<FeatureCloud, Eigen::aligned_allocator<FeatureCloud> > object_templates;
    vector<string> model_names_;
    FeatureCloud target_;
    vector<pcl::CorrespondencesPtr> template_scene_correspondences_;

    // TODO: create a set of models - object instances
    vector<ObjectHypothesis, Eigen::aligned_allocator<ObjectHypothesis> > object_hypotheses_;

    pcl::KdTreeFLANN<pcl::SHOT352> descr_matching;
    pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    string training_dir_;
    float cg_size_;
    float cg_thresh_;
    float rf_rad_;
    float threshold_accept_model_hypothesis_;
    int icp_max_iter_;
    float icp_corr_distance_;
    float icp_fitness_eps_;
    float hv_clutter_reg_;
    float hv_inlier_th_;
    float hv_rad_clutter_;
    float hv_regularizer_;
    bool hv_detect_clutter_;
};

#endif // RECOGNIZER_H
