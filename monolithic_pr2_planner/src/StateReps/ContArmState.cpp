#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <angles/angles.h>
#include <string>
#include <ros/console.h>

using namespace monolithic_pr2_planner;
using namespace angles;

boost::shared_ptr<RobotResolutionParams> ContArmState::m_params;

ContArmState::ContArmState() : 
    m_is_enforcing_joint_limits(true){
    if (!m_params){
        ROS_ERROR("Robot resolution parameters were not statically initialized!");
    }
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
