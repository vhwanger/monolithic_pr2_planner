#include <monolithic_pr2_planner/StateReps/RobotPose.h>

using namespace monolithic_pr2_planner;

bool RobotPose::operator==(const RobotPose& other){
    return (m_base_state == other.m_base_state &&
            m_right_arm == other.m_right_arm &&
            m_left_arm == other.m_left_arm);
}

bool RobotPose::operator!=(const RobotPose& other){
    return !(*this == other);
}

RobotPose::RobotPose(ContBaseState base_state, ContArmState r_arm, ContArmState l_arm):
    m_base_state(base_state), m_right_arm(r_arm), m_left_arm(l_arm){
}

ContBaseState RobotPose::getContBaseState(){
    return ContBaseState(m_base_state);    
}

DiscObjectState RobotPose::getMapFrameObjectState(){
    // TODO: make this arm agnostic?
    //m_right
}
