#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <boost/scoped_ptr.hpp>

using namespace monolithic_pr2_planner;
using namespace boost;


GraphState::GraphState(RobotPose robot_pose) : m_robot_pose(robot_pose){ }

GraphState::GraphState(DiscObjectState obj_state, RobotPose robot_pose):
 m_robot_pose(robot_pose){ }

bool GraphState::operator==(const GraphState& other){
    return (m_robot_pose.getDiscBaseState() == other.m_robot_pose.getDiscBaseState() &&
            m_robot_pose.getObjectStateRelBody() == other.m_robot_pose.getObjectStateRelBody() &&
            m_robot_pose.getLeftDiscFreeAngle() == other.m_robot_pose.getLeftDiscFreeAngle() &&
            m_robot_pose.getRightDiscFreeAngle() == other.m_robot_pose.getRightDiscFreeAngle());
}

bool GraphState::operator!=(const GraphState& other){
    return !(*this == other);
}


bool GraphState::applyMPrim(const GraphStateMotion& mprim){
    DiscObjectState obj_state = m_robot_pose.getObjectStateRelBody();
    obj_state.setX(obj_state.getX() + mprim[GraphStateElement::OBJ_X]);
    obj_state.setY(obj_state.getY() + mprim[GraphStateElement::OBJ_Y]);
    obj_state.setZ(obj_state.getZ() + mprim[GraphStateElement::OBJ_Z]);
    obj_state.setRoll(obj_state.getRoll() + mprim[GraphStateElement::OBJ_ROLL]);
    obj_state.setPitch(obj_state.getPitch() + mprim[GraphStateElement::OBJ_PITCH]);
    obj_state.setYaw(obj_state.getYaw() + mprim[GraphStateElement::OBJ_YAW]);

    DiscBaseState base_state = m_robot_pose.getDiscBaseState();
    base_state.setX(base_state.getX() + mprim[GraphStateElement::BASE_X]);
    base_state.setY(base_state.getY() + mprim[GraphStateElement::BASE_Y]);
    base_state.setTheta(base_state.getTheta() + mprim[GraphStateElement::BASE_THETA]);
    m_robot_pose.setDiscBaseState(base_state);

    RobotPosePtr new_robot_pose;
    if (RobotPose::computeRobotPose(obj_state, m_robot_pose, new_robot_pose)){
        m_robot_pose = *new_robot_pose;
    } else {
        return false;
    }
    return true;
}

void GraphState::printToDebug(char* logger) const {
    DiscObjectState obj_state = m_robot_pose.getObjectStateRelBody();
    ROS_DEBUG_NAMED(logger, "\t%d %d %d %d %d %d %d %d %d %d %d %d",
                    obj_state.getX(),
                    obj_state.getY(),
                    obj_state.getZ(),
                    obj_state.getRoll(),
                    obj_state.getPitch(),
                    obj_state.getYaw(),
                    m_robot_pose.getRightDiscFreeAngle(),
                    m_robot_pose.getLeftDiscFreeAngle(),
                    m_robot_pose.getDiscBaseState().getX(),
                    m_robot_pose.getDiscBaseState().getY(),
                    m_robot_pose.getDiscBaseState().getZ(),
                    m_robot_pose.getDiscBaseState().getTheta());
}

DiscObjectState GraphState::getObjectStateRelMap(){
    return m_robot_pose.getObjectStateRelMap();
}

DiscObjectState GraphState::getObjectStateRelBody(){
    return m_robot_pose.getObjectStateRelBody();
}
