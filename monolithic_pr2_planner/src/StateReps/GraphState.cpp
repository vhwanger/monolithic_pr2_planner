#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <boost/scoped_ptr.hpp>

using namespace monolithic_pr2_planner;
using namespace boost;

GraphState::GraphState(RobotState robot_pose) : m_robot_pose(robot_pose){ }

GraphState::GraphState(DiscObjectState obj_state, RobotState robot_pose):
 m_robot_pose(robot_pose){ }

bool GraphState::operator==(const GraphState& other){
    return (m_robot_pose.base_state() == other.m_robot_pose.base_state() &&
            m_robot_pose.getObjectStateRelBody() == other.m_robot_pose.getObjectStateRelBody() &&
            m_robot_pose.left_free_angle() == other.m_robot_pose.left_free_angle() &&
            m_robot_pose.right_free_angle() == other.m_robot_pose.right_free_angle());
}

bool GraphState::operator!=(const GraphState& other){
    return !(*this == other);
}

/*! \brief applies a generic mprim vector to this graph state.
 */
bool GraphState::applyMPrim(const GraphStateMotion& mprim){
    // object state change
    DiscObjectState obj_state = m_robot_pose.getObjectStateRelBody();
    obj_state.x(obj_state.x() + mprim[GraphStateElement::OBJ_X]);
    obj_state.y(obj_state.y() + mprim[GraphStateElement::OBJ_Y]);
    obj_state.z(obj_state.z() + mprim[GraphStateElement::OBJ_Z]);
    obj_state.roll(obj_state.roll() + mprim[GraphStateElement::OBJ_ROLL]);
    obj_state.pitch(obj_state.pitch() + mprim[GraphStateElement::OBJ_PITCH]);
    obj_state.yaw(obj_state.yaw() + mprim[GraphStateElement::OBJ_YAW]);
    DiscBaseState base_state = m_robot_pose.base_state();

    // free angle change
    RightContArmState right_arm = m_robot_pose.right_arm();
    int r_fa = right_arm.getDiscFreeAngle() + mprim[GraphStateElement::R_FA];
    right_arm.setDiscFreeAngle(r_fa);
    m_robot_pose.right_arm(right_arm);

    LeftContArmState left_arm = m_robot_pose.left_arm();
    int l_fa = left_arm.getDiscFreeAngle() + mprim[GraphStateElement::L_FA];
    left_arm.setDiscFreeAngle(l_fa);
    m_robot_pose.left_arm(left_arm);

    // base change
    base_state.x(base_state.x() + mprim[GraphStateElement::BASE_X]);
    base_state.y(base_state.y() + mprim[GraphStateElement::BASE_Y]);
    base_state.z(base_state.z() + mprim[GraphStateElement::BASE_Z]);
    base_state.theta(base_state.theta() + mprim[GraphStateElement::BASE_THETA]);
    m_robot_pose.base_state(base_state);

    // compute the new pose (runs IK)
    RobotPosePtr new_robot_pose;
    if (RobotState::computeRobotPose(obj_state, m_robot_pose, new_robot_pose)){
        m_robot_pose = *new_robot_pose;
    } else {
        return false;
    }
    return true;
}

void GraphState::printToDebug(char* logger) const {
    DiscObjectState obj_state = m_robot_pose.getObjectStateRelBody();
    DiscObjectState map_obj_state = m_robot_pose.getObjectStateRelMap();


    ROS_DEBUG_NAMED(logger, "\tobject in map %d %d %d %d %d %d",
                    map_obj_state.x(),
                    map_obj_state.y(),
                    map_obj_state.z(),
                    map_obj_state.roll(),
                    map_obj_state.pitch(),
                    map_obj_state.yaw());

    ROS_DEBUG_NAMED(logger, "\t%d %d %d %d %d %d %d %d %d %d %d %d",
                    obj_state.x(),
                    obj_state.y(),
                    obj_state.z(),
                    obj_state.roll(),
                    obj_state.pitch(),
                    obj_state.yaw(),
                    m_robot_pose.right_free_angle(),
                    m_robot_pose.left_free_angle(),
                    m_robot_pose.base_state().x(),
                    m_robot_pose.base_state().y(),
                    m_robot_pose.base_state().z(),
                    m_robot_pose.base_state().theta());
}

void GraphState::printContToDebug(char* logger) const {
    ContObjectState obj_state = m_robot_pose.getObjectStateRelBody();
    ContObjectState map_obj_state = m_robot_pose.getObjectStateRelMap();
    ContBaseState base_state = m_robot_pose.base_state();
    ROS_DEBUG_NAMED(logger, "object in map %f %f %f %f %f %f",
                    map_obj_state.x(),
                    map_obj_state.y(),
                    map_obj_state.z(),
                    map_obj_state.roll(),
                    map_obj_state.pitch(),
                    map_obj_state.yaw());
                    
    ROS_DEBUG_NAMED(logger, "\t%f %f %f %f %f %f %f %f %f %f %f %f",
                    obj_state.x(),
                    obj_state.y(),
                    obj_state.z(),
                    obj_state.roll(),
                    obj_state.pitch(),
                    obj_state.yaw(),
                    m_robot_pose.right_arm().getUpperArmRollAngle(),
                    m_robot_pose.left_arm().getUpperArmRollAngle(),
                    base_state.x(),
                    base_state.y(),
                    base_state.z(),
                    base_state.theta());
}

DiscObjectState GraphState::getObjectStateRelMap() const {
    return m_robot_pose.getObjectStateRelMap();
}

DiscObjectState GraphState::getObjectStateRelBody() const {
    return m_robot_pose.getObjectStateRelBody();
}

