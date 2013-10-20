#include <monolithic_pr2_planner/ArmModel.h>

using namespace monolithic_pr2_planner;
using namespace boost;

// throw some exceptions here??
ArmModel::ArmModel(HardwareDescriptionFiles robot_model){
    FILE* fp_l_arm = fopen(robot_model.l_arm_file.c_str(), "r");
    if (!fp_l_arm){
        ROS_ERROR("Couldn't open left arm model file (%s)!", 
                  robot_model.l_arm_file.c_str());
    }

    FILE* fp_r_arm = fopen(robot_model.r_arm_file.c_str(), "r");
    if (!fp_r_arm){
        ROS_ERROR("Couldn't open right arm model file (%s)!",
                   robot_model.r_arm_file.c_str());
    }

    m_l_arm = make_shared<sbpl_arm_planner::SBPLArmModel>(fp_l_arm);
    m_r_arm = make_shared<sbpl_arm_planner::SBPLArmModel>(fp_r_arm);

    m_l_arm->setResolution(robot_model.env_resolution);
    m_r_arm->setResolution(robot_model.env_resolution);

    if (!robot_model.robot_description_string.compare("ROS_PARAM")){
        ROS_INFO("getting kdl chain from paramserver");
        m_l_arm->initKDLChainFromParamServer(); 
        m_r_arm->initKDLChainFromParamServer();
    } else {
        ROS_INFO("getting kdl chain from string");
        m_l_arm->initKDLChain(robot_model.robot_description_string);
        m_r_arm->initKDLChain(robot_model.robot_description_string);
    }

}
