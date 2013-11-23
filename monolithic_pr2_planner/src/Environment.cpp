#include <monolithic_pr2_planner/Environment.h>
#include <monolithic_pr2_planner/OccupancyGridUser.h>
#include <monolithic_pr2_planner/StateReps/RobotState.h>
#include <monolithic_pr2_planner/StateReps/DiscBaseState.h>
#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <monolithic_pr2_planner/MotionPrimitives/ArmAdaptiveMotionPrimitive.h>
#include <monolithic_pr2_planner/Visualizer.h>
#include <monolithic_pr2_planner/Constants.h>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <assert.h>

#define GOAL_STATE 1
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
    assert(sourceStateID != GOAL_STATE);
    ROS_DEBUG_NAMED(SEARCH_LOG, 
            "==================Expanding state %d==================", 
                    sourceStateID);
    succIDs->clear();
    succIDs->reserve(m_mprims.getMotionPrims().size());
    costs->clear();
    costs->reserve(m_mprims.getMotionPrims().size());

    GraphStatePtr source_state = m_hash_mgr.getGraphState(sourceStateID);
    ROS_DEBUG_NAMED(SEARCH_LOG, "Source state is:");
    source_state->robot_pose().printToDebug(SEARCH_LOG);
    //source_state->robot_pose().visualize();
    //sleep(.1);

    for (auto mprim : m_mprims.getMotionPrims()){
        GraphStatePtr successor;
        TransitionData t_data;
        if (!mprim->apply(*source_state, successor, t_data)){
            continue;
        }
        if (m_cspace_mgr->isValidSuccessor(*successor, t_data) && 
            m_cspace_mgr->isValidTransitionStates(t_data)){
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
                succIDs->push_back(GOAL_STATE);
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


vector<RobotState> Environment::reconstructPath(const vector<int>& state_ids){
    vector<RobotState> path;
    auto all_mprims = m_mprims.getMotionPrims();
    vector<TransitionData> transition_states;
    // TODO reconstruct last goal state as well.
    for (size_t i=0; i < state_ids.size()-1; i++){
        int state_id = state_ids[i];
        GraphStatePtr source_state = m_hash_mgr.getGraphState(state_id);
        GraphStatePtr successor;
        int best_cost = 1000000;
        TransitionData best_transition;
        GraphStatePtr real_next_successor = m_hash_mgr.getGraphState(state_ids[i+1]);
        for (size_t i=0; i < all_mprims.size(); i++){
            TransitionData t_data;
            auto mprim = all_mprims[i];
            if (!mprim->apply(*source_state, successor, t_data)){
                continue;
            }
            bool isAdaptive = (t_data.motion_type() == MPrim_Types::BASE_ADAPTIVE || 
                               t_data.motion_type() == MPrim_Types::ARM_ADAPTIVE);
            if ((*successor != *real_next_successor) && !isAdaptive){
                continue;
            } else if (isAdaptive){
                ROS_DEBUG_NAMED(SEARCH_LOG, "found AMP in path reconstruction. adding it");
                best_transition = t_data;
                best_cost = 1;
                break;
            }
            if (t_data.cost() < best_cost){
                best_cost = t_data.cost();
                best_transition = t_data;
            }
        }
        if (best_cost != 1000000){
            transition_states.push_back(best_transition);
        } else {
            ROS_ERROR("Couldn't find the right successor???");
        }
    }
    
    printFinalPath(state_ids, transition_states);
    // TODO actually return path
    return path;
}

void Environment::printFinalPath(const vector<int>& state_ids,
                                 const vector<TransitionData>& transition_states){
    GraphStatePtr source_state = m_hash_mgr.getGraphState(state_ids[0]);
    source_state->robot_pose().visualize();
    source_state->robot_pose().printToInfo(SEARCH_LOG);
    sleep(.1);
    for (size_t i=0; i < transition_states.size(); i++){
        int motion_type = transition_states[i].motion_type();
        bool isInterpBaseMotion = (motion_type == MPrim_Types::BASE || 
                                   motion_type == MPrim_Types::BASE_ADAPTIVE);
        if (isInterpBaseMotion){
            ROS_DEBUG_NAMED(SEARCH_LOG, "found an interpolated base motion of size %lu", 
                                        transition_states[i].cont_base_interm_steps().size());
            for (size_t j=0; j < transition_states[i].cont_base_interm_steps().size(); j++){
                RobotState robot = transition_states[i].interm_robot_steps()[j];
                vector<double> l_arm, r_arm;
                robot.right_arm().getAngles(&r_arm);
                robot.left_arm().getAngles(&l_arm);
                ContBaseState cont_base = transition_states[i].cont_base_interm_steps()[j];
                cont_base.printToDebug(SEARCH_LOG);
                BodyPose bp = cont_base.body_pose();
                Visualizer::pviz->visualizeRobot(r_arm, l_arm, bp, 150, "planner", 0);
                usleep(50000);
            } 
        } else {
            if (motion_type == MPrim_Types::BASE)
                ROS_DEBUG_NAMED(SEARCH_LOG, "reconstructing BASE");
            if (motion_type == MPrim_Types::ARM)
                ROS_DEBUG_NAMED(SEARCH_LOG, "reconstructing ARM");
            if (motion_type == MPrim_Types::BASE_ADAPTIVE)
                ROS_DEBUG_NAMED(SEARCH_LOG, "reconstructing BASE_ADAPTIVE");
            if (motion_type == MPrim_Types::ARM_ADAPTIVE)
                ROS_DEBUG_NAMED(SEARCH_LOG, "reconstructing ARM_ADAPTIVE");
            if (motion_type == MPrim_Types::TORSO)
                ROS_DEBUG_NAMED(SEARCH_LOG, "reconstructing TORSO");
            for (size_t j=0; j < transition_states[i].interm_robot_steps().size(); j++){
                RobotState robot = transition_states[i].interm_robot_steps()[j];
                robot.printToDebug(SEARCH_LOG);
                robot.visualize();
                usleep(50000);
            }
        }

        source_state = m_hash_mgr.getGraphState(state_ids[i+1]);
        source_state->robot_pose().visualize();
        source_state->robot_pose().printToInfo(SEARCH_LOG);
        usleep(50000);
    }
    GraphStatePtr soln_state = m_goal->getSolnState();
    soln_state->robot_pose().visualize();
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
