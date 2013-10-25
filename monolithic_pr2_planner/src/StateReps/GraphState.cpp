#include <monolithic_pr2_planner/StateReps/GraphState.h>

using namespace monolithic_pr2_planner;


GraphState::GraphState(RobotPose robot_pose) : 
m_robot_pose(robot_pose){
    m_obj_state = m_robot_pose.getMapFrameObjectState();
}

bool GraphState::operator==(const GraphState& other){
    return (m_robot_pose.getDiscBaseState() == other.m_robot_pose.getDiscBaseState() &&
            m_obj_state == other.m_obj_state &&
            m_robot_pose.getLeftDiscFreeAngle() == other.m_robot_pose.getLeftDiscFreeAngle() &&
            m_robot_pose.getRightDiscFreeAngle() == other.m_robot_pose.getRightDiscFreeAngle());
}

bool GraphState::operator!=(const GraphState& other){
    return !(*this == other);
}
