#include <monolithic_pr2_planner/MotionPrimitives/BaseMotionPrimitive.h>
#include <monolithic_pr2_planner/Constants.h>
#include <assert.h>
#include <angles/angles.h>

// TODO WTF is this
#define METER_TO_MM_MULT 1000

using namespace monolithic_pr2_planner;
using namespace boost;
using namespace std;
using namespace angles;

void BaseMotionPrimitive::print() const {
    ROS_DEBUG_NAMED(MPRIM_LOG, "Base Primitive");
    ROS_DEBUG_NAMED(MPRIM_LOG, "\tid: %d", getID());
    ROS_DEBUG_NAMED(MPRIM_LOG, "\tstart angle: %d", start_angle());
    ROS_DEBUG_NAMED(MPRIM_LOG, "\tcost: %d", getCost());
    printEndCoord();
    printIntermSteps();
}


bool BaseMotionPrimitive::apply(const GraphState& source_state, 
                           GraphStatePtr& successor){
    // since the base motion primitive list contains motion primitives for every
    // possible base theta, let's only use the one corresponding to our
    // particular angle.
    if (source_state.robot_pose().base_state().theta() != start_angle()){
        return false;
    }
    successor.reset(new GraphState(source_state));
    return successor->applyMPrim(m_end_coord);
}

// computes cost of a motion primitive as the max between the linear distance
// traveled and the angular distance of the turn
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

    double turn_time_rads = static_cast<double>(params.turn_45_deg_in_place_time)*M_PI/4.0;
    double angular_time = angular_distance/turn_time_rads;
    //double angular_time = angular_distance/((M_PI/4.0)/
    //params.turn_45_deg_in_place_time);

    //make the cost the max of the two times
    m_cost = ceil(static_cast<double>(METER_TO_MM_MULT)*(max(linear_time, angular_time)));
    //use any additional cost multiplier
    m_cost *= getAdditionalCostMult();
    assert(m_cost != 0.0);
}

