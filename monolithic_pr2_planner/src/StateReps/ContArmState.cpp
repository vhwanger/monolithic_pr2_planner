#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <angles/angles.h>
#include <string>
#include <ros/console.h>

using namespace monolithic_pr2_planner;
using namespace angles;
using namespace std;

boost::shared_ptr<RobotResolutionParams> ContArmState::m_params;
SBPLArmModelPtr ContArmState::m_arm_model;
KDL::Frame ContArmState::m_object_offset;
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

unsigned int ContArmState::getDiscFreeAngle() const {
    double free_angle_res = m_params->arm_free_angle_resolution;
    double free_angle = m_angles[Joints::UPPER_ARM_ROLL];
    unsigned int disc_angle = (unsigned int)((normalize_angle_positive(free_angle + 
                                              free_angle_res*0.5))/free_angle_res);
    assert(free_angle == convertDiscFreeAngleToCont(disc_angle));
    return disc_angle;
}

double ContArmState::convertDiscFreeAngleToCont(unsigned int disc_angle) const {
    double free_angle_res = m_params->arm_free_angle_resolution;
    return normalize_angle_positive(double(disc_angle)*free_angle_res);
}

void ContArmState::getAngles(std::vector<double>* angles){
    *angles = m_angles;
}

//template<class Derived>
//void BaseX<Derived>::setArmModel(ArmDescriptionParams& params){
//    FILE* fp_arm= fopen(params.arm_file.c_str(), "r");
//    if (!fp_arm){
//        ROS_ERROR("Couldn't open right arm model file (%s)!",
//                   params.arm_file.c_str());
//    }
//    m_arm_model = boost::make_shared<sbpl_arm_planner::SBPLArmModel>(fp_arm);
//    m_arm_model->setResolution(params.env_resolution);
//    if (!params.robot_description_string.compare("ROS_PARAM")){
//        ROS_INFO("getting kdl chain from paramserver");
//        m_arm_model->initKDLChainFromParamServer();
//    } else {
//        ROS_INFO("getting kdl chain from string");
//        m_arm_model->initKDLChain(params.robot_description_string);
//    }
//}

