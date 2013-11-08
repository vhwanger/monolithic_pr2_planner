#include <monolithic_pr2_planner/StateReps/GoalState.h>

using namespace monolithic_pr2_planner;

GoalState::GoalState(SearchRequestPtr search_request, HeuristicPtr heur):
    m_goal_state(search_request->m_params->obj_goal), m_tolerances(4,0),
    m_heur(heur) {
    // TODO: should i run an IK check here just to test feasibility?
    heur->setGoal(m_goal_state); 
    m_tolerances[Tolerances::XYZ] = search_request->m_params->xyz_tolerance;
    m_tolerances[Tolerances::ROLL] = search_request->m_params->roll_tolerance;
    m_tolerances[Tolerances::PITCH] = search_request->m_params->pitch_tolerance;
    m_tolerances[Tolerances::YAW] = search_request->m_params->yaw_tolerance;
}

bool GoalState::isSatisfiedBy(const GraphStatePtr& graph_state){
    // not sure why there's a .005 here. ask ben
    ContObjectState c_tol(m_tolerances[Tolerances::XYZ]-.005, 
                          m_tolerances[Tolerances::XYZ]-.005, 
                          m_tolerances[Tolerances::XYZ]-.005,
                          m_tolerances[Tolerances::ROLL],
                          m_tolerances[Tolerances::PITCH],
                          m_tolerances[Tolerances::YAW]);
    DiscObjectState d_tol = c_tol.getDiscObjectState();
    DiscObjectState obj = graph_state->getObjectStateRelMap();

    bool within_xyz_tol = (abs(m_goal_state.getX()-obj.getX()) < d_tol.getX() &&
                           abs(m_goal_state.getY()-obj.getY()) < d_tol.getY() &&
                           abs(m_goal_state.getZ()-obj.getZ()) < d_tol.getZ());
    bool within_rpy_tol = (abs(m_goal_state.getRoll()-obj.getRoll()) < d_tol.getRoll() &&
                           abs(m_goal_state.getPitch()-obj.getPitch()) < d_tol.getPitch() &&
                           abs(m_goal_state.getYaw()-obj.getYaw()) < d_tol.getYaw());

    if (within_xyz_tol && within_rpy_tol){
        return true;
    } else {
        return false;
    }
}

bool GoalState::isSolnStateID(int state_id){
    for (auto& goal : m_possible_goals){
        if (goal == state_id){
            return true;
        }
    }
    return false;
}
void GoalState::addPotentialSolnState(const GraphStatePtr& graph_state) { 
    m_possible_goals.push_back(graph_state->getID());
}
