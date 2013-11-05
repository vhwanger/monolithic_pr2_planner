#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <angles/angles.h>
#include <string>
#include <ros/console.h>

using namespace monolithic_pr2_planner;
using namespace angles;
using namespace std;

boost::shared_ptr<RobotResolutionParams> ContArmState::m_params;
KDL::Frame LeftContArmState::m_object_offset;
KDL::Frame RightContArmState::m_object_offset;
SBPLArmModelPtr LeftContArmState::m_arm_model;
SBPLArmModelPtr RightContArmState::m_arm_model;

bool ContArmState::operator==(const ContArmState& other){
    return (m_angles == other.m_angles);
}

bool ContArmState::operator!=(const ContArmState& other){
    return !(*this == other);
}

ContArmState::ContArmState() : 
    m_is_enforcing_joint_limits(true), m_angles(7,0){
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

int ContArmState::getDiscFreeAngle() const {
    double free_angle_res = m_params->arm_free_angle_resolution;
    double free_angle = m_angles[Joints::UPPER_ARM_ROLL];
    int disc_angle = static_cast<int>((normalize_angle_positive(free_angle + 
                                              free_angle_res*0.5))/free_angle_res);
    assert(free_angle-convertDiscFreeAngleToCont(disc_angle)<free_angle_res);
    return disc_angle;
}

void ContArmState::getAngles(std::vector<double>* angles) const {
    *angles = m_angles;
}


void LeftContArmState::initArmModel(ArmDescriptionParams& params){
    FILE* fp_arm= fopen(params.arm_file.c_str(), "r");
    if (!fp_arm){
        ROS_ERROR("Couldn't open right arm model file (%s)!",
                   params.arm_file.c_str());
    }
    SBPLArmModelPtr arm_model = boost::make_shared<sbpl_arm_planner::SBPLArmModel>(fp_arm);
    arm_model->setResolution(params.env_resolution);
    if (!params.robot_description_string.compare("ROS_PARAM")){
        ROS_INFO("getting kdl chain from paramserver");
        arm_model->initKDLChainFromParamServer();
    } else {
        ROS_INFO("getting kdl chain from string");
        arm_model->initKDLChain(params.robot_description_string);
    }
    m_arm_model = arm_model;
}

void RightContArmState::initArmModel(ArmDescriptionParams& params){
    FILE* fp_arm= fopen(params.arm_file.c_str(), "r");
    if (!fp_arm){
        ROS_ERROR("Couldn't open right arm model file (%s)!",
                   params.arm_file.c_str());
    }
    SBPLArmModelPtr arm_model = boost::make_shared<sbpl_arm_planner::SBPLArmModel>(fp_arm);
    arm_model->setResolution(params.env_resolution);
    if (!params.robot_description_string.compare("ROS_PARAM")){
        ROS_INFO("getting kdl chain from paramserver");
        arm_model->initKDLChainFromParamServer();
    } else {
        ROS_INFO("getting kdl chain from string");
        arm_model->initKDLChain(params.robot_description_string);
    }
    m_arm_model = arm_model;
}

DiscObjectState ContArmState::getObjectStateRelBody(){
    // don't remember what 10 is for. ask ben.
    KDL::Frame to_wrist;
    getArmModel()->computeArmFK(m_angles, 10, &to_wrist);
    double roll1,pitch1,yaw1;
    to_wrist.M.GetRPY(roll1,pitch1,yaw1);
    KDL::Frame f = to_wrist * getObjectOffset().Inverse();

    double wr,wp,wy;
    f.M.GetRPY(wr,wp,wy);

    ContObjectState cont_obj_state(f.p.x(), f.p.y(), f.p.z(), wr, wp, wy);
    return DiscObjectState(cont_obj_state);
}
