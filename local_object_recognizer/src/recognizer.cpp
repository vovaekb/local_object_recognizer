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
#include <pcl/registration/transformation_estimation_svd.h>

#include "local_object_recognizer/common.h"
#include "local_object_recognizer/feature_cloud.h"
#include "local_object_recognizer/persistence_utils.h"
#include "local_object_recognizer/recognizer.h"

Recognizer::Recognizer() :
    cg_size_ (cg_size),
    cg_thresh_ (cg_threshold),
    rf_rad_ (rf_radius),
    threshold_accept_model_hypothesis_(0.3f),
    icp_max_iter_ (icp_max_iter),
    icp_corr_distance_ (icp_corr_distance),
    icp_fitness_eps_ (icp_fitness_eps),
    hv_clutter_reg_ (hv_clutter_reg),
    hv_inlier_th_ (hv_inlier_thresh),
    hv_rad_clutter_ (hv_rad_clutter),
    hv_regularizer_ (hv_regularizer),
    hv_detect_clutter_ (false)
{
    gc_clusterer.setGCSize(cg_size_);
    gc_clusterer.setGCThreshold(cg_thresh_);
    icp.setMaxCorrespondenceDistance(icp_corr_distance_);
    icp.setMaximumIterations(icp_max_iter_);
}

Recognizer::~Recognizer() {}

void Recognizer::setTrainingDir(const string &training_dir)
{
    training_dir_ = training_dir;
}

void Recognizer::setTargetCloud(FeatureCloud &target_cloud)
{
    target_ = target_cloud;
    gc_clusterer.setSceneCloud(target_cloud.getKeypoints());
    icp.setInputTarget(target_cloud.getPointCloud());
}

void Recognizer::setRFRadius(const float &rf_radius)
{
    rf_rad_ = rf_radius;
}

void Recognizer::setCGSize(const float &cg_s)
{
    cg_size_ = cg_s;
    gc_clusterer.setGCSize(cg_size_);
}

void Recognizer::setCGThresh(const float &cg_th)
{
    cg_thresh_ = cg_th;
    gc_clusterer.setGCThreshold(cg_thresh_);
}

void Recognizer::setHVInlierThresh(const float &hv_thresh)
{
    hv_inlier_th_ = hv_thresh;
}

vector<Recognizer::ObjectHypothesis, Eigen::aligned_allocator<Recognizer::ObjectHypothesis> > Recognizer::getModels()
{
    return object_hypotheses_;
}

vector<string> Recognizer::getTrainingModelNames()
{
    return model_names_;
}

void Recognizer::addTemplateCloud(FeatureCloud &template_cloud)
{
    object_templates.push_back(template_cloud);
}

void Recognizer::addModelId(const string &model_name)
{
    model_names_.push_back(model_name);
}

void Recognizer::printModels()
{
    cout << "Recognizer.model_names:\n";

    for(int i = 0; i < model_names_.size(); i++)
    {
        cout << " * " << model_names_.at(i) << "\n";
    }
}

void Recognizer::match()
{
    cout << "\n\n---------------- Matching ------------------\n\n";

    // A kd-tree object that uses the FLANN library for fast search of nearest neighbors.
    pcl::KdTreeFLANN<SHOTDescriptorType> shot_matching;

    template_scene_correspondences_.clear();

    for(size_t i = 0; i < object_templates.size(); i++)
    {
        cout << "\n\nMatching for model template " << i << "\n";
        shot_matching.setInputCloud(object_templates[i].getLocalFeatures());
        // A Correspondence object stores the indices of the query and the match,
        // and the distance/weight.
        pcl::CorrespondencesPtr correspondences (new pcl::Correspondences());

        // Check every descriptor computed for the scene
        for(size_t j = 0; j < target_.getLocalFeatures()->size(); ++j)
        {
            vector<int> neighbors(1);
            vector<float> squared_distances(1);
            // Ignore NaNs.
            if(pcl_isfinite(target_.getLocalFeatures()->at(j).descriptor[0]))
            {
                int k = 1; // number of neighbors
                neighbors.resize(k);
                squared_distances.resize(k);
                // Find the nearest neighbor (in descriptor space) ...
                int neighbor_count = shot_matching.nearestKSearch(target_.getLocalFeatures()->at(j), k, neighbors, squared_distances);
                // ...and add a new correspondence if the distance is less than a threshold
                // (SHOT distances are between 0 and 1).
                if(neighbor_count == 1 && squared_distances[0] <  0.25f)
                {
                    pcl::Correspondence correspondence(neighbors[0], static_cast<int>(j), squared_distances[0]);
                    correspondences->push_back(correspondence);
                }
            }
        }

        cout << "[Recognizer::match] Found " << correspondences->size() << " correspondences\n";

        template_scene_correspondences_.push_back(correspondences);
    }
}

void Recognizer::group_template_correspondences(FeatureCloud &model_template, const int index)
{
    vector<pcl::Correspondences> corresp_clusters;

    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transformations;

    PointCloudTypePtr template_keypoints = model_template.getKeypoints();
    pcl::PointCloud<NormalType>::Ptr template_normals = model_template.getSurfaceNormals();
    PointCloudTypePtr template_cloud = model_template.getPointCloud();

    // Geometric consistency
    if(!use_hough)
    {
//        cout << "Perform GC algorithm\n";

        gc_clusterer.setInputCloud(model_template.getKeypoints());
        gc_clusterer.setSceneCloud(target_.getKeypoints());


        gc_clusterer.setModelSceneCorrespondences(template_scene_correspondences_[index]);

        gc_clusterer.cluster(corresp_clusters);
    }
    else
    {
//        cout << "Perform Hough voting algorithm\n";

        // Hough Voting
        //
        //  Compute (Keypoints) Reference Frames only for Hough
        //
        pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
        pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

        pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
        rf_est.setFindHoles (true);
        rf_est.setRadiusSearch (rf_rad_);

        rf_est.setInputCloud (template_keypoints);
        rf_est.setInputNormals (template_normals);
        rf_est.setSearchSurface (template_cloud);
        rf_est.compute (*model_rf);

        rf_est.setInputCloud (target_.getKeypoints());
        rf_est.setInputNormals (target_.getSurfaceNormals());
        rf_est.setSearchSurface (target_.getPointCloud());
        rf_est.compute (*scene_rf);

        //  Clustering
        pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
        clusterer.setHoughBinSize (cg_size_);
        clusterer.setHoughThreshold (cg_thresh_);
        clusterer.setUseInterpolation (true);
        clusterer.setUseDistanceWeight (false);

        clusterer.setInputCloud (template_keypoints);
        clusterer.setInputRf (model_rf);
        clusterer.setSceneCloud (target_.getKeypoints());
        clusterer.setSceneRf (scene_rf);

        clusterer.setModelSceneCorrespondences (template_scene_correspondences_[index]);

        clusterer.cluster(corresp_clusters);
    }

    cout << "Clusters for model " << index << " found: " << corresp_clusters.size() << "\n";

    // Correspondence cardinality based rejection step
    std::vector<bool> good_indices_for_hypotheses (corresp_clusters.size(), true);

    // sort the hypotheses for each model according to their correspondences and take those that are threshold_accept_model_hypothesis_ over the max cardinality
    int max_cardinality = -1;
    for(size_t i = 0; i < corresp_clusters.size(); i++)
    {
        if(max_cardinality < static_cast<int> (corresp_clusters[i].size()))
        {
            max_cardinality = static_cast<int> (corresp_clusters[i].size());
        }
    }

    for(size_t i = 0; i < corresp_clusters.size(); i++)
    {
        if(static_cast<float> (corresp_clusters[i].size()) < (threshold_accept_model_hypothesis_ * static_cast<float> (max_cardinality)))
        {
            good_indices_for_hypotheses[i] = false;
        }
    }


    cout << "Perform TransformationEstimationSVD\n";

    for(size_t i = 0; i < corresp_clusters.size(); i++)
    {
        if(!good_indices_for_hypotheses[i])
            continue;

        Eigen::Matrix4f best_trans;
        pcl::registration::TransformationEstimationSVD <PointType, PointType> t_est;
        t_est.estimateRigidTransformation(*template_keypoints, *(target_.getKeypoints()), corresp_clusters[i], best_trans);
        transformations.push_back(best_trans);
    }

    cout << "Clusters survived after the cardinality rejection: " << transformations.size() << "\n\n\n";


    for(size_t j = 0; j < transformations.size(); j++)
    {
        ObjectHypothesis oh;
        oh.model_template = model_template;
        oh.transformation = transformations[j];
        object_hypotheses_.push_back(oh);
    }

}

void Recognizer::group_correspondences()
{
    cout << "\n---------------- Correspondences grouping --------------------\n";

    for(size_t i = 0; i < object_templates.size(); i++)
    {
        cout << "CG algorithm for object template: " << i << "\n";

        group_template_correspondences(object_templates[i], i);
    }
}


void Recognizer::alignAll()
{
    cout << "-------------- ICP -----------------\n";

    cout << "Number of hypotheses: " << object_hypotheses_.size() << "\n";

    for(size_t i = 0; i < object_hypotheses_.size(); i++)
    {
        ObjectHypothesis oh = object_hypotheses_.at(i);

        cout << "Alignment of the object hypotheses " << i <<  " - " << oh.model_template.getModelId() << "_" << oh.model_template.getViewId() << "\n";

        PointCloudTypeConstPtr template_cloud = oh.model_template.getPointCloud();
        PointCloudTypePtr template_aligned (new pcl::PointCloud<PointType> ());
        pcl::transformPointCloud(*template_cloud, *template_aligned, oh.transformation);

        pcl::IterativeClosestPoint<PointType, PointType> icp;

        // RANSAC
        if(perform_ransac)
        {
            pcl::registration::CorrespondenceRejectorSampleConsensus<PointType>::Ptr rej (new pcl::registration::CorrespondenceRejectorSampleConsensus<PointType> ());
            rej->setInputTarget(target_.getPointCloud());
            rej->setMaximumIterations (sac_max_iters);
            rej->setInlierThreshold(sac_inlier_thresh);
            rej->setInputSource(template_aligned);

            icp.addCorrespondenceRejector(rej);
        }

        icp.setMaximumIterations(icp_max_iter_);
        icp.setInputTarget(target_.getPointCloud());
        icp.setInputSource(template_aligned);
        PointCloudTypePtr registered (new pcl::PointCloud<PointType> ());
        icp.align(*registered);

        Eigen::Matrix4f icp_trans = icp.getFinalTransformation();
        oh.transformation = icp_trans * oh.transformation;

        float score = icp.getFitnessScore();

        oh.icp_score = score;

        cout << "Fitness score: " << score << "\n";
    }

    cout << "\n-------------\n\n";
}

void Recognizer::recognize()
{
    cout << "Recognize objects in scene ...\n";

    object_hypotheses_.clear();

    match();

    group_correspondences();

    alignAll();

    cout << "------------------- Hypotheses verification ------------------------\n";

    if(!object_hypotheses_.size())
    {
        cout << "No object hypotheses found\n";
        return;
    }

    vector<bool> hypotheses_mask; // Mask vector to identify positive hypotheses

    vector<PointCloudTypeConstPtr> aligned_templates;

    for(int i = 0; i < object_hypotheses_.size(); i++)
    {
        ObjectHypothesis oh = object_hypotheses_.at(i);
        PointCloudTypeConstPtr template_cloud = oh.model_template.getPointCloud();
        PointCloudTypePtr template_aligned (new pcl::PointCloud<PointType>);
        pcl::transformPointCloud(*template_cloud, *template_aligned, oh.transformation);

        aligned_templates.push_back(template_aligned);
    }

    pcl::GlobalHypothesesVerification<PointType, PointType> hyp_ver;

    hyp_ver.setSceneCloud(target_.getPointCloud());
    hyp_ver.addModels(aligned_templates, true);

    hyp_ver.setInlierThreshold(hv_inlier_th_);
    hyp_ver.setRegularizer(hv_regularizer_);
    hyp_ver.setRadiusClutter(hv_rad_clutter_);
    hyp_ver.setClutterRegularizer(hv_clutter_reg_);
    hyp_ver.setDetectClutter(hv_detect_clutter_);

    hyp_ver.verify();
    hyp_ver.getMask(hypotheses_mask);

    vector<ObjectHypothesis, Eigen::aligned_allocator<ObjectHypothesis> > templates_temp;

    for(int i = 0; i < hypotheses_mask.size(); i++)
    {
//        cout << "Hypothesis " << i;
        if(hypotheses_mask[i])
        {
//            cout << " is GOOD! \n";

            templates_temp.push_back(object_hypotheses_[i]);
        }
        else
        {
//            cout << " is bad!\n";
        }

    }

    cout << "-------------------\n";

    // Order found hypotheses by ICP score
    sort(templates_temp.begin(), templates_temp.end(), sortIndexScoresOp);

    // For every model find the best hypothesis
    object_hypotheses_.clear();

    for(int i = 0; i < model_names_.size(); i++)
    {
        string model_name = model_names_[i];

        cout << "Search for the best hypothesis for the model: " << model_name << "\n";

        for(int j = 0; j < templates_temp.size(); j++)
        {
            ObjectHypothesis oh = templates_temp[j];

            if(oh.model_template.getModelId() == model_name)
            {
                object_hypotheses_.push_back(oh);
                break;
            }
        }
    }

}


