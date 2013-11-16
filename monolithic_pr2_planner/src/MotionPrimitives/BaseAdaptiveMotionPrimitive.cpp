#include <monolithic_pr2_planner/MotionPrimitives/BaseAdaptiveMotionPrimitive.h>
#include <monolithic_pr2_planner/StateReps/RobotState.h>
#include <monolithic_pr2_planner/StateReps/ContBaseState.h>
#include <monolithic_pr2_planner/StateReps/ContObjectState.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <cmath>
#include <vector>
#include <assert.h>

using namespace monolithic_pr2_planner;
using namespace boost;
using namespace std;

GoalState BaseAdaptiveMotionPrimitive::m_goal;

BaseAdaptiveMotionPrimitive::BaseAdaptiveMotionPrimitive(int direction):
    m_direction(direction){ }

// TODO fix variable names 
bool BaseAdaptiveMotionPrimitive::apply(const GraphState& source_state, 
                                         GraphStatePtr& successor){
    DiscObjectState d_goal = m_goal.getObjectState();
    ContObjectState c_goal = m_goal.getObjectState();
    ContObjectState cur_obj_state = source_state.getObjectStateRelMap();
    if (dist(cur_obj_state, d_goal) > 2){
        return false;
    }

    ROS_DEBUG_NAMED(MPRIM_LOG, "source state");
    source_state.printToDebug(MPRIM_LOG);
    source_state.robot_pose().visualize();
    ROS_DEBUG_NAMED(MPRIM_LOG, "orig object state yaw: %f", cur_obj_state.yaw());
    ROS_DEBUG_NAMED(MPRIM_LOG, "goal state");
    d_goal.printToDebug(MPRIM_LOG);
    c_goal.printToDebug(MPRIM_LOG);

    ContBaseState base_state = source_state.robot_pose().base_state();
    ROS_DEBUG_NAMED(MPRIM_LOG, "original continuous base: %f %f %f %f", 
                               base_state.x(),
                               base_state.y(),
                               base_state.z(),
                               base_state.theta());
    double del_c_theta = c_goal.yaw()-cur_obj_state.yaw();
    int sign = static_cast<int>(del_c_theta/fabs(del_c_theta));
    assert(abs(sign) == 1);
    int del_theta = DiscBaseState::convertContTheta(fabs(del_c_theta));
    ROS_DEBUG_NAMED(MPRIM_LOG, "Turned the base by %f (or %d theta steps).", 
                    del_c_theta, del_theta);

    DiscBaseState d_base_state = source_state.robot_pose().base_state();
    d_base_state.theta(d_base_state.theta() + sign*del_theta);
    ROS_DEBUG_NAMED(MPRIM_LOG, "The new base theta is now %d", d_base_state.theta());
    RobotState base_turned_pose(d_base_state, 
                                source_state.robot_pose().right_arm(), 
                                source_state.robot_pose().left_arm());
    ROS_DEBUG_NAMED(MPRIM_LOG, "Object theta should be closer to the goal's theta now.");
    ContObjectState c_unadjusted_obj = base_turned_pose.getObjectStateRelMap();
    ContBaseState new_base = base_turned_pose.base_state();
    ROS_DEBUG_NAMED(MPRIM_LOG, "New base theta: %f", new_base.theta());
    c_unadjusted_obj.printToDebug(MPRIM_LOG);
    ROS_DEBUG_NAMED(MPRIM_LOG, "Now need to readjust the object state back "
                               "to the original xyz pose by translating the base.");
    double c_dx = c_goal.x() - c_unadjusted_obj.x();
    double c_dy = c_goal.y() - c_unadjusted_obj.y();
    int sign_x = static_cast<int>(c_dx/fabs(c_dx));
    int sign_y = static_cast<int>(c_dy/fabs(c_dy));
    int dx = DiscBaseState::convertContDistance(fabs(c_dx));
    int dy = DiscBaseState::convertContDistance(fabs(c_dy));
    ROS_DEBUG_NAMED(MPRIM_LOG, "Need to move base by %f (%d) %f (%d)",
                    c_dx, dx, c_dy, dy);
    DiscBaseState new_base_state = base_turned_pose.base_state();
    new_base_state.x(new_base_state.x() + sign_x*dx);
    new_base_state.y(new_base_state.y() + sign_y*dy);
    ContBaseState c_new_base_state = new_base_state;
    ROS_DEBUG_NAMED(MPRIM_LOG, "new base state is now %f %f %f (%d %d %d", 
                                c_new_base_state.x(), 
                                c_new_base_state.y(),
                                c_new_base_state.z(),
                                new_base_state.x(),
                                new_base_state.y(),
                                new_base_state.z());

                                
    RobotState seed_state(new_base_state, source_state.robot_pose().right_arm(),
                                          source_state.robot_pose().left_arm());

    RobotPosePtr new_robot_state;

    DiscObjectState obj_in_body_frame = source_state.getObjectStateRelBody();
    obj_in_body_frame.roll(c_goal.roll());
    obj_in_body_frame.pitch(c_goal.pitch());
    ContBaseState c_base = new_base_state;

    obj_in_body_frame.yaw(c_goal.yaw()-c_unadjusted_obj.yaw());


    bool isIKSuccess = RobotState::computeRobotPose(obj_in_body_frame, seed_state, 
                                                    new_robot_state);
    if (!isIKSuccess){
        return false;
    }
    ContObjectState final_obj_state = new_robot_state->getObjectStateRelMap();
    ROS_DEBUG_NAMED(MPRIM_LOG, "new obj state is now %f %f %f %f %f %f", 
                                final_obj_state.x(), 
                                final_obj_state.y(),
                                final_obj_state.z(),
                                final_obj_state.roll(),
                                final_obj_state.pitch(),
                                final_obj_state.yaw());
    successor = make_shared<GraphState>(*new_robot_state);
    successor->robot_pose().visualize();


    //print();
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
