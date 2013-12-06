#include <monolithic_pr2_planner/MotionPrimitives/TorsoMotionPrimitive.h>

using namespace monolithic_pr2_planner;

TorsoMotionPrimitive::TorsoMotionPrimitive(int vertical_direction){
    m_end_coord[GraphStateElement::OBJ_Z] = vertical_direction;
    m_end_coord[GraphStateElement::BASE_Z] = vertical_direction;
}

bool TorsoMotionPrimitive::apply(const GraphState& source_state,
                                 GraphStatePtr& successor,
                                 TransitionData& t_data){
    successor.reset(new GraphState(source_state));
    bool isSuccessorCreated = successor->applyMPrim(m_end_coord);
    if (isSuccessorCreated){
        t_data.motion_type(motion_type());
        t_data.cost(cost());
    }
    // TODO pr2cc should be doing this check instead
    ContBaseState base_state = successor->robot_pose().base_state();
    if (base_state.z() < 0 || base_state.z() > .31){
        return false;
    }
    return isSuccessorCreated;
}

void TorsoMotionPrimitive::print() const {
    ROS_DEBUG_NAMED(MPRIM_LOG, "Torso Primitive");
    ROS_DEBUG_NAMED(MPRIM_LOG, "\tcost: %d", cost());
    printEndCoord();
}

void TorsoMotionPrimitive::computeCost(const MotionPrimitiveParams& params){
    m_cost = 3000;
}
