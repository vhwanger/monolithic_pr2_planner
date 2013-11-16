#include <monolithic_pr2_planner/MotionPrimitives/ArmAdaptiveMotionPrimitive.h>
#include <monolithic_pr2_planner/StateReps/DiscObjectState.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <monolithic_pr2_planner/Visualizer.h>
#include <cmath>

using namespace monolithic_pr2_planner;
using namespace std;

GoalState ArmAdaptiveMotionPrimitive::m_goal;

bool ArmAdaptiveMotionPrimitive::apply(const GraphState& source_state,
                                  GraphStatePtr& successor){
    DiscObjectState goal = m_goal.getObjectState();
    // TODO parameterize this distance?
    if (dist(source_state.getObjectStateRelMap(), goal) > 2){
        return false;
    }

    DiscBaseState base_state = source_state.robot_pose().base_state();
    RobotState seed_pose(base_state, 
                         source_state.robot_pose().right_arm(), 
                         source_state.robot_pose().left_arm());
    RobotPosePtr successor_robot_pose;
    
    DiscObjectState obj_in_body_frame = source_state.getObjectStateRelBody();

    obj_in_body_frame.roll(goal.roll());
    obj_in_body_frame.pitch(goal.pitch());

    // take the discrete orientation (theta) of the robot and convert it
    // into the discrete yaw in the object frame. object frame yaw is
    // generally discretized into 64 states, whereas the base theta is 16.
    ContBaseState c_base = source_state.robot_pose().base_state();
    ContObjectState c_obj_state = m_goal.getObjectState();
    c_obj_state.yaw(c_base.theta());
    DiscObjectState temp = c_obj_state;
    obj_in_body_frame.yaw(goal.yaw()-temp.yaw());

    bool isIKSuccess = RobotState::computeRobotPose(obj_in_body_frame, 
                                                    seed_pose, 
                                                    successor_robot_pose);
    if (isIKSuccess){
        ROS_DEBUG_NAMED(SEARCH_LOG, "successful arm adaptive motion!");
        successor = boost::make_shared<GraphState>(*successor_robot_pose);
    } else {
        ROS_DEBUG_NAMED(SEARCH_LOG, "IK failed on arm AMP");
    }
    return isIKSuccess;
}


void ArmAdaptiveMotionPrimitive::print() const {

}

void ArmAdaptiveMotionPrimitive::computeCost(const MotionPrimitiveParams& params){
    m_cost = 1;
}
