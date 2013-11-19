#include <monolithic_pr2_planner/Environment.h>
#include <monolithic_pr2_planner/OccupancyGridUser.h>
#include <monolithic_pr2_planner/StateReps/RobotState.h>
#include <monolithic_pr2_planner/StateReps/DiscBaseState.h>
#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <monolithic_pr2_planner/MotionPrimitives/ArmAdaptiveMotionPrimitive.h>
#include <monolithic_pr2_planner/Visualizer.h>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <assert.h>

using namespace monolithic_pr2_planner;
using namespace boost;


typedef scoped_ptr<SearchRequest> SearchRequestPtr;

// TODO yuck @ stateid2mapping pointer
Environment::Environment(ros::NodeHandle nh) : 
    m_hash_mgr(&StateID2IndexMapping), m_nodehandle(nh), m_mprims(m_goal){
    m_param_catalog.fetch(nh);
    configurePlanningDomain();
}

bool Environment::configureRequest(SearchRequestParamsPtr search_request_params,
                                   int& start_id, int& goal_id){
    SearchRequestPtr search_request = SearchRequestPtr(new SearchRequest(search_request_params));
    configureQuerySpecificParams(search_request);
    if (!setStartGoal(search_request, start_id, goal_id)){
        return false;
    }

    return true;
}

int Environment::GetGoalHeuristic(int stateID){
    return m_heur->getGoalHeuristic(m_hash_mgr.getGraphState(stateID));
}

void Environment::GetSuccs(int sourceStateID, vector<int>* succIDs, 
                           vector<int>* costs){
    ROS_DEBUG_NAMED(SEARCH_LOG, 
            "==================Expanding state %d==================", 
                    sourceStateID);
    if (sourceStateID == 1){
        ROS_DEBUG_NAMED(SEARCH_LOG, "expanding goal state?");
        return;
    }
    succIDs->clear();
    succIDs->reserve(m_mprims.getMotionPrims().size());
    costs->clear();
    costs->reserve(m_mprims.getMotionPrims().size());

    GraphStatePtr source_state = m_hash_mgr.getGraphState(sourceStateID);
    ROS_DEBUG_NAMED(SEARCH_LOG, "Source state is:");
    source_state->robot_pose().printToDebug(SEARCH_LOG);
    source_state->robot_pose().visualize();
    sleep(.1);

    if (m_heur->getGoalHeuristic(source_state) < 5){
        ROS_DEBUG_NAMED(SEARCH_LOG, "super close to goal");
    }


    for (auto mprim : m_mprims.getMotionPrims()){
        GraphStatePtr successor;
        if (m_cspace_mgr->isValidMotion(*source_state, mprim, successor)){
            ROS_DEBUG_NAMED(SEARCH_LOG, 
            "==================Applying Motion Primitive==================");
            ROS_DEBUG_NAMED(MPRIM_LOG, "Applying motion:");
            mprim->printEndCoord();
            m_hash_mgr.save(successor);
            ROS_DEBUG_NAMED(MPRIM_LOG, "successor state with id %d is:", 
                            successor->id());
            successor->printToDebug(MPRIM_LOG);

            if (m_goal->isSatisfiedBy(successor)){
                m_goal->storeAsSolnState(successor);
                ROS_INFO_NAMED(SEARCH_LOG, "Found potential goal");
                // TODO define the goal state somewhere
                succIDs->push_back(1);
            } else {
                succIDs->push_back(successor->id());
            }
            costs->push_back(1);
            //costs->push_back(mprim->cost());
        } 
    }
}

bool Environment::setStartGoal(SearchRequestPtr search_request,
                               int& start_id, int& goal_id){
    RobotState start_pose(search_request->m_params->base_start, 
                         search_request->m_params->right_arm_start,
                         search_request->m_params->left_arm_start);
    ContObjectState obj_state = start_pose.getObjectStateRelMap();
    obj_state.printToInfo(SEARCH_LOG);

    if (!search_request->isValid(m_cspace_mgr)){
        obj_state.printToInfo(SEARCH_LOG);
        start_pose.visualize();
        return false;
    }

    GraphStatePtr start_graph_state = make_shared<GraphState>(start_pose);
    m_hash_mgr.save(start_graph_state);
    start_id = start_graph_state->id();
    assert(m_hash_mgr.getGraphState(start_graph_state->id()) == start_graph_state);

    ROS_INFO_NAMED(SEARCH_LOG, "Start state set to:");
    start_pose.printToInfo(SEARCH_LOG);
    obj_state.printToInfo(SEARCH_LOG);
    start_pose.visualize();


    m_goal = make_shared<GoalState>(search_request, m_heur);

    // TODO fix this shit! shouldn't be saving the start state twice.
    // also, api fail.
    GraphStatePtr fake_goal = make_shared<GraphState>(*start_graph_state);
    fake_goal->robot_pose().base_state().x(0);
    fake_goal->robot_pose().base_state().y(0);
    fake_goal->robot_pose().base_state().z(0);
    RobotState blah_robot_state = fake_goal->robot_pose();
    DiscBaseState blah_base = blah_robot_state.base_state();
    blah_base.x(0); blah_base.y(0); blah_base.z(0);
    blah_robot_state.base_state(blah_base);
    fake_goal->robot_pose(blah_robot_state);
    ROS_INFO("base state %d", 
             fake_goal->robot_pose().base_state().x());
    m_hash_mgr.save(fake_goal);
    goal_id = fake_goal->id();

    ROS_INFO_NAMED(SEARCH_LOG, "Goal state created:");
    ContObjectState c_goal = m_goal->getObjectState();
    c_goal.printToInfo(SEARCH_LOG);
    m_goal->visualize();

    // TODO yuck
    ArmAdaptiveMotionPrimitive::goal(*m_goal);
    BaseAdaptiveMotionPrimitive::goal(*m_goal);

    return true;
}


// this sets up the environment for things that are query independent.
void Environment::configurePlanningDomain(){
    // used for collision space and discretizing plain xyz into grid world 
    OccupancyGridUser::init(m_param_catalog.m_occupancy_grid_params,
                            m_param_catalog.m_robot_resolution_params);

    // used for discretization of robot movements
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
    
    // load heuristic
    m_heur = make_shared<Heuristic>(m_cspace_mgr);

    // load up motion primitives
    m_mprims.loadMPrims(m_param_catalog.m_motion_primitive_params);

    // load up static pviz instance for visualizations. 
    //boost::shared_ptr<PViz> m_pviz = boost::make_shared<PViz>();
    //m_pviz->setReferenceFrame("map");
    //RobotState::setPViz(m_pviz);
    Visualizer::createPVizInstance();
    Visualizer::setReferenceFrame(std::string("/map"));

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



bool Environment::InitializeMDPCfg(MDPConfig *MDPCfg) {
  //MDPCfg->goalstateid = envMobileArm.goalHashEntry->stateID;
  //MDPCfg->startstateid = envMobileArm.startHashEntry->stateID;
  return true;
}

int Environment::SizeofCreatedEnv() {
    return m_hash_mgr.size();
}

void Environment::PrintState(int stateID, bool bVerbose, FILE* fOut /*=NULL*/) {

}
