#include <monolithic_pr2_planner/MotionPrimitives/BaseMotionPrimitive.h>
#include <monolithic_pr2_planner/Constants.h>
#include <assert.h>
#include <angles/angles.h>

#define METER_TO_MM_MULT 1000

using namespace monolithic_pr2_planner;
using namespace boost;
using namespace std;
using namespace angles;

void BaseMotionPrimitive::print() const {
    ROS_DEBUG_NAMED(MPRIM_LOG, "Base Primitive");
    ROS_DEBUG_NAMED(MPRIM_LOG, "\tid: %d", getID());
    ROS_DEBUG_NAMED(MPRIM_LOG, "\tstart angle: %d", start_angle());
    ROS_DEBUG_NAMED(MPRIM_LOG, "\tcost: %d", cost());
    printEndCoord();
    printIntermSteps();
}

/*! \brief given the graph state, this applies the base motion primitive while
 * filling in the transitiondata information.  since the base motion primitive
 * list contains motion primitives for every possible base theta, let's only use
 * the one corresponding to our particular angle.
 */
bool BaseMotionPrimitive::apply(const GraphState& source_state, 
                           GraphStatePtr& successor,
                           TransitionData& t_data){
    // skip irrelevant angles
    if (source_state.robot_pose().base_state().theta() != start_angle()){
        return false;
    }
    successor.reset(new GraphState(source_state));
    DiscBaseState base_state = successor->robot_pose().base_state();

    base_state.x(base_state.x() + m_end_coord[GraphStateElement::BASE_X]);
    base_state.y(base_state.y() + m_end_coord[GraphStateElement::BASE_Y]);
    base_state.z(base_state.z() + m_end_coord[GraphStateElement::BASE_Z]);
    base_state.theta(base_state.theta() + m_end_coord[GraphStateElement::BASE_THETA]);
    RobotState new_state(base_state, successor->robot_pose().right_arm(), successor->robot_pose().left_arm());
    successor->robot_pose(new_state);

    t_data.motion_type(motion_type());
    t_data.cost(cost());
    vector<RobotState> interm_robot_steps;
    vector<ContBaseState> cont_base_interm_steps;
    // TODO make sure this skips the first and last points in the intermediate
    // steps list - they are repeats of the start and end position
    //ROS_DEBUG_NAMED(MPRIM_LOG, "Creating BaseMotionPrimitive intermediate steps");
    for (auto interm_mprim_steps : getIntermSteps()){
        RobotState robot_state = source_state.robot_pose();
        ContBaseState interm_base = robot_state.base_state();
        interm_base.x(interm_base.x() + interm_mprim_steps[GraphStateElement::BASE_X]);
        interm_base.y(interm_base.y() + interm_mprim_steps[GraphStateElement::BASE_Y]);

        // we don't add the original theta like the above because BASE_THETA
        // represents the absolute angle, not the delta angle.
        interm_base.theta(interm_mprim_steps[GraphStateElement::BASE_THETA]);
        robot_state.base_state(interm_base);
        interm_robot_steps.push_back(robot_state);
        cont_base_interm_steps.push_back(interm_base);

    }
    GraphState last_state(interm_robot_steps[interm_robot_steps.size()-1]);
    assert(*successor == last_state);
    t_data.interm_robot_steps(interm_robot_steps);
    t_data.cont_base_interm_steps(cont_base_interm_steps);
    assert(t_data.cont_base_interm_steps().size() == t_data.interm_robot_steps().size());
    return true;
}

/*! \brief computes cost of a motion primitive as the max between the linear distance
 *  traveled and the angular distance of the turn
 */
void BaseMotionPrimitive::computeCost(const MotionPrimitiveParams& params){
    double linear_distance = 0;

    IntermSteps steps = getIntermSteps();

    for (size_t i=1; i < steps.size(); i++){
        double x0 = steps[i-1][GraphStateElement::BASE_X];
        double y0 = steps[i-1][GraphStateElement::BASE_Y];
        double x1 = steps[i][GraphStateElement::BASE_X];
        double y1 = steps[i][GraphStateElement::BASE_Y];
        double dx = x1-x0;
        double dy = y1-y0;
        linear_distance += sqrt(dx*dx + dy*dy);
    }
    double linear_time = linear_distance/static_cast<double>(params.nominal_vel);
    double first_angle = steps[0][GraphStateElement::BASE_THETA];
    double final_angle = steps.back()[GraphStateElement::BASE_THETA];
    double angular_distance = fabs(shortest_angular_distance(first_angle, 
                                                             final_angle));
    assert((linear_time > 0) || (angular_distance > 0));

    double angular_time = angular_distance/params.angular_vel;

    //make the cost the max of the two times
    m_cost = ceil(static_cast<double>(METER_TO_MM_MULT)*(max(linear_time, angular_time)));
    //use any additional cost multiplier
    m_cost *= getAdditionalCostMult();
    assert(m_cost >= 0.0);
}

