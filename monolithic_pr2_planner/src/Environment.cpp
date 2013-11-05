#include <monolithic_pr2_planner/Environment.h>
#include <monolithic_pr2_planner/OccupancyGridUser.h>
#include <monolithic_pr2_planner/StateReps/RobotPose.h>
#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <pviz/pviz.h>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/foreach.hpp>
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
    configureQuerySpecificParams(search_request);
    if (!setStartGoal(search_request)){
        return false;
    }
    

    // testing
    vector<int> succ, costs, actions;
    GetSuccs(0, &succ, &costs, &actions);

    return true;
}

void Environment::GetSuccs(int sourceStateID, vector<int>* succIDs, 
                           vector<int>* costs, vector<int>* actions){
    GraphStatePtr source_state = m_hash_mgr.getGraphState(sourceStateID);
    vector<unique_ptr<GraphState> > valid_successors;
    BOOST_FOREACH(auto mprim, m_mprims.getMotionPrims()){
        unique_ptr<GraphState> successor;
        if (m_cspace_mgr->isValidMotion(*source_state, mprim, successor)){
            ROS_DEBUG_NAMED(SEARCH_LOG, "source state:");
            source_state->printToDebug(SEARCH_LOG);
            ROS_DEBUG_NAMED(SEARCH_LOG, "successor state:");
            successor->printToDebug(SEARCH_LOG);

            valid_successors.push_back(std::move(successor));
            costs->push_back(mprim->getCost());
        } else {
            ROS_DEBUG_NAMED(SEARCH_LOG, "failed successor");
        }
    }
}

bool Environment::setStartGoal(SearchRequestPtr search_request){
    RobotPose start_pose(search_request->m_params->base_start, 
                         search_request->m_params->right_arm_start,
                         search_request->m_params->left_arm_start);
    ContObjectState obj_state = start_pose.getObjectStateRelMap();

    if (!search_request->isValid(m_cspace_mgr)){
        obj_state.printToInfo(SEARCH_LOG);
        start_pose.visualize();
        return false;
    }

    GraphStatePtr start_graph_state = make_shared<GraphState>(start_pose);
    m_hash_mgr.save(start_graph_state);
    assert(m_hash_mgr.getGraphState(start_graph_state->getID()) == start_graph_state);

    ROS_INFO_NAMED(SEARCH_LOG, "Start state set to:");
    start_pose.printToInfo(SEARCH_LOG);
    obj_state.printToInfo(SEARCH_LOG);
    start_pose.visualize();

    GoalState goal(search_request);
    m_goals.push_back(goal);

    ROS_INFO_NAMED(SEARCH_LOG, "Goal state created:");
    goal.getContObjectState().printToInfo(SEARCH_LOG);

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
    LeftContArmState::initArmModel(m_param_catalog.m_left_arm_params);
    RightContArmState::initArmModel(m_param_catalog.m_right_arm_params);

    // collision space mgr needs arm models in order to do collision checking
    // have to do this funny thing  of initializing an object because of static
    // variable + inheritance (see ContArmState for details)
    LeftContArmState l_arm;
    RightContArmState r_arm;
    m_cspace_mgr = make_shared<CollisionSpaceMgr>(l_arm.getArmModel(),
                                                  r_arm.getArmModel());

    // load up motion primitives
    m_mprims.loadMPrims(m_param_catalog.m_motion_primitive_files);

    // load up static pviz instance for visualizations. 
    boost::shared_ptr<PViz> pviz = boost::make_shared<PViz>();
    pviz->setReferenceFrame("map");
    RobotPose::setPViz(pviz);
}

// sets parameters for query specific things
void Environment::configureQuerySpecificParams(SearchRequestPtr search_request){
    // sets the location of the object in the frame of the wrist
    // have to do this funny thing  of initializing an object because of static
    // variable + inheritance (see ContArmState for details)
    LeftContArmState l_arm;
    RightContArmState r_arm;
    l_arm.setObjectOffset(search_request->m_params->left_arm_object);
    r_arm.setObjectOffset(search_request->m_params->right_arm_object);
}
