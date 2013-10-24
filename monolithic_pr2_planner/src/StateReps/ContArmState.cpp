#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <angles/angles.h>
#include <string>
#include <ros/console.h>

using namespace monolithic_pr2_planner;
using namespace angles;
using namespace std;

boost::shared_ptr<RobotResolutionParams> ContArmState::m_params;
SBPLArmModelPtr LeftContArmState::m_arm_model;
SBPLArmModelPtr RightContArmState::m_arm_model;

bool ContArmState::operator==(const ContArmState& other){
    return (m_angles == other.m_angles);
}

bool ContArmState::operator!=(const ContArmState& other){
    return !(*this == other);
}

ContArmState::ContArmState() : 
    m_is_enforcing_joint_limits(true){
    if (!m_params){
        ROS_ERROR("Robot resolution parameters were not statically initialized!");
    }
}

ContArmState::ContArmState(vector<double> arm_state) : 
    m_angles(arm_state) {
}

void ContArmState::setRobotResolutionParams(const RobotResolutionParams& params){
    m_params = boost::make_shared<RobotResolutionParams>(params);
}

unsigned int ContArmState::getDiscFreeAngle(){
    double free_angle_res = m_params->arm_free_angle_resolution;
    double free_angle = m_angles[Joints::UPPER_ARM_ROLL];
    unsigned int disc_angle = (unsigned int)((normalize_angle_positive(free_angle + 
                                              free_angle_res*0.5))/free_angle_res);
    assert(free_angle == convertDiscFreeAngleToCont(disc_angle));
    return disc_angle;
}

double ContArmState::convertDiscFreeAngleToCont(unsigned int disc_angle){
    double free_angle_res = m_params->arm_free_angle_resolution;
    return normalize_angle_positive(double(disc_angle)*free_angle_res);
}

void ContArmState::getVectorOfAngles(std::vector<double>* angles){
    *angles = m_angles;
}

boost::shared_ptr<ContArmState> ArmStateFactory::createArmState(int arm_side, 
                                                         std::vector<double> angles){
    // TODO: clean up the signature of arm_side
    if (arm_side == ArmSide::LEFT){
        return boost::shared_ptr<ContArmState>(new LeftContArmState(angles));
    } else {
        return boost::shared_ptr<ContArmState>(new RightContArmState(angles));
    }
}


void RightContArmState::setArmModel(const HardwareDescriptionFiles& params){
    FILE* fp_r_arm = fopen(params.r_arm_file.c_str(), "r");
    if (!fp_r_arm){
        ROS_ERROR("Couldn't open right arm model file (%s)!",
                   params.r_arm_file.c_str());
    }
    m_arm_model = boost::make_shared<sbpl_arm_planner::SBPLArmModel>(fp_r_arm);
    m_arm_model->setResolution(params.env_resolution);
    if (!params.robot_description_string.compare("ROS_PARAM")){
        ROS_INFO("getting kdl chain from paramserver");
        m_arm_model->initKDLChainFromParamServer();
    } else {
        ROS_INFO("getting kdl chain from string");
        m_arm_model->initKDLChain(params.robot_description_string);
    }
}


void LeftContArmState::setArmModel(const HardwareDescriptionFiles& params){
    FILE* fp_l_arm = fopen(params.l_arm_file.c_str(), "r");
    if (!fp_l_arm){
        ROS_ERROR("Couldn't open left arm model file (%s)!", 
                  params.l_arm_file.c_str());
    }
    m_arm_model = boost::make_shared<sbpl_arm_planner::SBPLArmModel>(fp_l_arm);
    m_arm_model->setResolution(params.env_resolution);
    if (!params.robot_description_string.compare("ROS_PARAM")){
        ROS_INFO("getting kdl chain from paramserver");
        m_arm_model->initKDLChainFromParamServer(); 
    } else {
        ROS_INFO("getting kdl chain from string");
        m_arm_model->initKDLChain(params.robot_description_string);
    }
}
