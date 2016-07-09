#ifndef CONSOLE_PARSE_H
#define CONSOLE_PARSE_H

#include <string>

namespace ConsoleUtils
{
    void showHelp()
    {
        std::string filename = "rosrun local_object_recognizer recognizer";
        std::cout << "\n";
        std::cout << "***************************************************************************\n";
        std::cout << "*                                                                         *\n";
        std::cout << "*                      RGB-D object recognizer - Usage Guide                *\n";
        std::cout << "*                                                                         *\n";
        std::cout << "***************************************************************************\n";
        std::cout << "Usage " << filename << " _models_dir:=<models_dir> _gt_dir:=<ground_truth_dir> [options]\n";
        std::cout << "\t<ground_truth_dir>:                      Path to ground truth files\n";
        std::cout << "Options are:\n";
        std::cout << "\t_h=true:                                 Show this help.\n";
        std::cout << "_scene_pcd:=<cloud.pcd>                    Scene point cloud (normal mode only)\n";
        std::cout << "\t_model:                                  Model name to be used instead of reading models file\n";
        std::cout << "\t_ransac:=true                            Use RANSAC in ICP step\n";
        std::cout << "\t_rf_radius:=<rf_radius>                  RF radius\n";
        std::cout << "\t_cg_size:=<cg_size>                      Correspondence grouping size\n";
        std::cout << "\t_cg_thresh:=<cg_thresh>                  Correspondence grouping threshold\n";
        std::cout << "\t_vis:=true :                             Visualize recognition results\n";
    }

    void parseCommandLine()
    {
        ros::NodeHandle private_node_handle_("~");

        bool show_help(false);
        private_node_handle_.getParam("h", show_help);

        if(show_help)
        {
            std::cout << "Show help\n";
            showHelp();
            exit(0);
        }

        private_node_handle_.getParam("models_dir", models_dir);

        if(models_dir == "")
        {
            ROS_ERROR("Models directory missing.");
            showHelp();
            exit(-1);
        }

        private_node_handle_.getParam("scene_pcd", scene_pcd_file);

        private_node_handle_.getParam("gt_files_dir", gt_files_dir);

        private_node_handle_.getParam("ransac", perform_ransac);

        private_node_handle_.getParam("rf_radius", rf_radius);

        private_node_handle_.getParam("cg_size", cg_size);

        private_node_handle_.getParam("cg_thresh", cg_threshold);

        private_node_handle_.getParam("vis", visualize);

        private_node_handle_.getParam("model", model_name);
    }
}

#endif // CONSOLE_PARSE_H
