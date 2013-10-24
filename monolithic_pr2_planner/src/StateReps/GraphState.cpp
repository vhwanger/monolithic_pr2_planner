#include <monolithic_pr2_planner/StateReps/GraphState.h>

using namespace monolithic_pr2_planner;


GraphState::GraphState(RobotPose robot_pose) : 
m_robot_pose(robot_pose){
    m_disc_obj_state = m_robot_pose.getDiscObjectState();
}

bool GraphState::operator==(const GraphState& other){
    return (m_robot_state.getDiscBaseState() == other.m_robot_state.getDiscBaseState() &&
            m_disc_obj_state == other.m_disc_obj_state &&
            m_robot_state.getLeftDiscFreeAngle() == other.getLeftDiscFreeAngle() &&
            m_robot_state.getRightDiscFreeAngle() == other.getRightDiscFreeAngle());
}

bool GraphState::operator!=(const GraphState& other){
    return !(*this == other);
}
