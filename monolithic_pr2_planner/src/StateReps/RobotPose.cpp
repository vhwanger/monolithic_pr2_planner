#include <monolithic_pr2_planner/StateReps/RobotPose.h>

using namespace monolithic_pr2_planner;

RobotPose::RobotPose(ContBaseState base_state, ContArmState r_arm, ContArmState l_arm):
    m_base_state(base_state), m_right_arm(r_arm), m_left_arm(l_arm){
}

RobotPose::getContBaseState(){
}
