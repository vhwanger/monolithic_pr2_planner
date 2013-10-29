#include <monolithic_pr2_planner/Environment.h>
#include <monolithic_pr2_planner/OccupancyGridUser.h>
#include <monolithic_pr2_planner/StateReps/RobotPose.h>
#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <pviz/pviz.h>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <assert.h>

using namespace monolithic_pr2_planner;
using namespace boost;


typedef scoped_ptr<SearchRequest> SearchRequestPtr;
Environment::Environment(ros::NodeHandle nh) : m_nodehandle(nh){
    m_param_catalog.fetch(nh);
    configurePlanningDomain();
}

bool Environment::plan(SearchRequestParamsPtr search_request_params){
    SearchRequestPtr search_request = SearchRequestPtr(new SearchRequest(search_request_params));
    if (!setStartGoal(search_request))
        return false;

    return true;
}

bool Environment::setStartGoal(SearchRequestPtr search_request){
    RobotPose start_pose(search_request->m_params->base_start, 
                         search_request->m_params->right_arm_start,
                         search_request->m_params->left_arm_start);
    ContObjectState obj_state = start_pose.getDiscMapFrameObjectState();
    if (!search_request->isValid(m_collision_space_mgr)){
        obj_state.printToInfo(INIT_LOG);
        start_pose.visualize();
        return false;
    }
    configureQuerySpecificParams(search_request);
    GraphStatePtr start_graph_state = make_shared<GraphState>(start_pose);
    m_hash_mgr.save(start_graph_state);
    assert(m_hash_mgr.getGraphState(start_graph_state->getID()) == start_graph_state);

    ROS_INFO_NAMED(SEARCH_LOG, "Start state set to:");
    start_pose.printToInfo(SEARCH_LOG);
    obj_state.printToInfo(SEARCH_LOG);
    start_pose.visualize();

    GoalStateParams goal_params;
    goal_params.goal_state = search_request->m_params->obj_goal;
    goal_params.xyz_tolerance = search_request->m_params->xyz_tolerance;
    goal_params.roll_tolerance = search_request->m_params->roll_tolerance;
    goal_params.pitch_tolerance = search_request->m_params->pitch_tolerance;
    goal_params.yaw_tolerance = search_request->m_params->yaw_tolerance;
    GoalState goal(goal_params);
    m_goals.push_back(goal);

    ROS_INFO_NAMED(SEARCH_LOG, "Goal state created:");
    goal_params.goal_state.printToInfo(SEARCH_LOG);

    return true;
}


// this sets up the environment for things that are query independent.
void Environment::configurePlanningDomain(){
    // used for collision space and discretizing functions 
    OccupancyGridUser::init(m_param_catalog.m_occupancy_grid_params,
                            m_param_catalog.m_robot_resolution_params);

    // used for arm discretization
    ContArmState::setRobotResolutionParams(m_param_catalog.m_robot_resolution_params);

    // used for arm kinematics
    LeftContArmState::setArmModel(m_param_catalog.m_left_arm_params);
    RightContArmState::setArmModel(m_param_catalog.m_right_arm_params);

    // collision space mgr needs arm models in order to do collision checking
    m_collision_space_mgr = make_shared<CollisionSpaceMgr>(LeftContArmState::getArmModel(),
                                                           RightContArmState::getArmModel());

    // load up static pviz instance for visualizations. 
    boost::shared_ptr<PViz> pviz = boost::make_shared<PViz>();
    pviz->setReferenceFrame("map");
    RobotPose::setPViz(pviz);
}

// sets parameters for query specific things
void Environment::configureQuerySpecificParams(SearchRequestPtr search_request){
    // sets the location of the object in the frame of the wrist
    LeftContArmState::setObjectOffset(search_request->m_params->left_arm_object);
    RightContArmState::setObjectOffset(search_request->m_params->right_arm_object);
}
