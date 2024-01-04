#ifndef COMMON_H
#define COMMON_H

#include <string>
#include "local_object_recognizer/recognizer.h"

constexpr bool perform_verification;
constexpr bool visualize;
constexpr bool use_hough;
constexpr bool use_hv;

constexpr bool perform_ransac;
// Algorithm params
constexpr float descr_rad;
constexpr double rf_radius;
constexpr double cg_size;
constexpr double cg_threshold;

constexpr int sac_max_iters;
constexpr const int icp_max_iter;
constexpr float sac_inlier_thresh;
constexpr float hv_inlier_thresh;

// Training scene and models paths
constexpr std::string scene_pcd_file;
constexpr std::string scene_name;
constexpr std::string model_name;
constexpr std::string models_dir;
constexpr std::string model_descr_dir;
constexpr std::string base_descr_dir;
constexpr std::string gt_files_dir;
constexpr string gt_file_path;

constexpr const float icp_corr_distance;
constexpr const float icp_fitness_eps;
constexpr const float hv_clutter_reg;
constexpr const float hv_rad_clutter;
constexpr const float hv_regularizer;

constexpr Recognizer recognizer;

#endif // COMMON_H
