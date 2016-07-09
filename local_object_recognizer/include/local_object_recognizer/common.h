#ifndef COMMON_H
#define COMMON_H

#include <string>
#include "local_object_recognizer/recognizer.h"

extern bool perform_verification;
extern bool visualize;
extern bool use_hough;
extern bool use_hv;

extern bool perform_ransac;
// Algorithm params
extern float descr_rad;
extern double rf_radius;
extern double cg_size;
extern double cg_threshold;

extern int sac_max_iters;
extern const int icp_max_iter;
extern float sac_inlier_thresh;
extern float hv_inlier_thresh;


// Training scene and models paths
extern std::string scene_pcd_file;
extern std::string scene_name;
extern std::string model_name;
extern std::string models_dir;
extern std::string model_descr_dir;
extern std::string base_descr_dir;
extern std::string gt_files_dir;
extern string gt_file_path;

extern const float icp_corr_distance;
extern const float icp_fitness_eps;
extern const float hv_clutter_reg;
extern const float hv_rad_clutter;
extern const float hv_regularizer;

extern Recognizer recognizer;

#endif // COMMON_H
