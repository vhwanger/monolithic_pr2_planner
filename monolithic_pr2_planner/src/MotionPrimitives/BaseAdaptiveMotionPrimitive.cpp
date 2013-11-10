#include <monolithic_pr2_planner/MotionPrimitives/BaseAdaptiveMotionPrimitive.h>
#include <monolithic_pr2_planner/StateReps/RobotState.h>
#include <monolithic_pr2_planner/StateReps/ContBaseState.h>
#include <monolithic_pr2_planner/StateReps/ContObjectState.h>
#include <cmath>
#include <vector>

using namespace monolithic_pr2_planner;
using namespace boost;
using namespace std;

BaseAdaptiveMotionPrimitive::BaseAdaptiveMotionPrimitive(int direction,
                                                         const MotionPrimitiveParams& params): 
    m_direction(direction), m_params(params){ }

bool BaseAdaptiveMotionPrimitive::apply(const GraphState& source_state, 
                                         GraphStatePtr& successor){
    ROS_INFO("generating base adaptive mprim stuff");
    DiscBaseState base_state = source_state.robot_pose().base_state();
    ROS_INFO("base state theta %d", base_state.theta());
    base_state.theta(base_state.theta() + m_direction);
    RobotState after_orbit_pose(base_state, 
                               source_state.robot_pose().right_arm(), 
                               source_state.robot_pose().left_arm());
    ROS_INFO("base state theta %d", after_orbit_pose.base_state().theta());
    ROS_INFO("successor");
    after_orbit_pose.printToDebug(SEARCH_LOG);
    RobotPosePtr successor_robot_pose;
    if (RobotState::computeRobotPose(source_state.getObjectStateRelBody(), 
                                    after_orbit_pose, successor_robot_pose)){
        return false;
    } else {
        successor = make_shared<GraphState>(*successor_robot_pose);
        computeIntermSteps(source_state, *successor);
    }
    print();
    return true;
}

// TODO: add to this comment
// Since these intermediate steps are computed at every step, the values
// generated here are absolute coordinates, NOT delta coordinates.
void BaseAdaptiveMotionPrimitive::computeIntermSteps(const GraphState& source_state,
                                                     const GraphState& successor){
    int num_steps = 2.0*M_PI/180 * (m_direction); // TODO check every two degrees
    ContBaseState start_base_state = source_state.robot_pose().base_state();
    ContBaseState end_base_state = source_state.robot_pose().base_state();

    vector<ContBaseState> interp_base_states;
    ContBaseState::interpolate(start_base_state, end_base_state, 
                               num_steps, &interp_base_states);
    vector<double> obj_state_coord;

    // TODO API fail
    double r_free_angle = source_state.robot_pose().right_arm().getUpperArmRollAngle();
    double l_free_angle = source_state.robot_pose().left_arm().getUpperArmRollAngle();

    ContObjectState obj_state = source_state.getObjectStateRelMap();
    obj_state.getStateValues(&obj_state_coord);
    for(auto& base_step : interp_base_states){
        vector<double> step;

        // concat everything together into a vector holding GraphState values
        step.insert(step.end(), obj_state_coord.begin(), obj_state_coord.end());
        step.push_back(r_free_angle);
        step.push_back(l_free_angle);
        vector<double> base_vector;
        base_step.getValues(&base_vector);
        step.insert(step.end(), base_vector.begin(), base_vector.end());
        m_interm_steps.push_back(step);
    }
}

void BaseAdaptiveMotionPrimitive::print() const {
    ROS_DEBUG_NAMED(CONFIG_LOG, "Base Adaptive Primitive");
    ROS_DEBUG_NAMED(CONFIG_LOG, "\tcost: %d", getCost());
    printEndCoord();
    printIntermSteps();
}

void BaseAdaptiveMotionPrimitive::computeCost(const MotionPrimitiveParams& params){
    // TODO find the right value;
    m_cost = 1;
}
