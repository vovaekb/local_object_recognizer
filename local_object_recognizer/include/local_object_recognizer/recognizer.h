#ifndef RECOGNIZER_H
#define RECOGNIZER_H

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET

#include <list>
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

#include "local_object_recognizer/typedefs.h"

using namespace std;

class Recognizer
{
public:
    // A struct for storing alignment results
    struct ObjectHypothesis
    {
        ObjectHypothesis(ObjectHypothesis &&o)
        {
            copyPointCloud(*(o.model_template), *model_template);
            transformation = std::move(o.transformation);
            icp_score = f.icp_score;

            o.model_template.reset(new PointCloud);
            o.transformation.resize(0, 0);
            o.icp_score = 0;
        }
        FeatureCloud model_template;
        float icp_score;
        Eigen::Matrix4f transformation;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    using ObjectHypothesisAllocator = Eigen::aligned_allocator<ObjectHypothesis>;

    Recognizer();

    ~Recognizer();

    void setTrainingDir(const string &training_dir);

    void setTargetCloud(FeatureCloud &target_cloud);

    void setRFRadius(const float &rf_radius);

    void setCGSize(const float &cg_s);

    void setCGThresh(const float &cg_th);

    void setHVInlierThresh(const float &hv_thresh);

    std::vector<ObjectHypothesis, ObjectHypothesisAllocator> getModels();

    vector<string> getTrainingModelNames();

    void addTemplateCloud(FeatureCloud &template_cloud);

    void addModelId(const string &model_name);

    void printModels();

    void matchObjectTemplate(FeatureCloud &obj_template, SHOTDescriptorKdTree shot_matching);

    // Matching
    void match();

    // CG for one template
    void group_template_correspondences(FeatureCloud &template_cloud, const int index);

    // CG over all templates
    void group_correspondences();

    // HV
    void hypotheses_verification();

    // ICP alignment over all templates
    void alignAll();

    // Find the best object hypotheses and their 6DOF poses
    void recognize();

private:
    FeatureCloudVector object_templates;
    std::vector<string> model_names_;
    FeatureCloud target_;
    CorrespondencesPtrVector template_scene_correspondences_;

    // TODO: create a set of models - object instances
    std::vector<ObjectHypothesis, ObjectHypothesisAllocator> object_hypotheses_;

    pcl::KdTreeFLANN<SHOTDescriptorType> descr_matching;
    pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
    pcl::IterativeClosestPoint<PointType, PointType>::Ptr icp;
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
