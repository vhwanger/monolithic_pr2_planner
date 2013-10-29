#include <monolithic_pr2_planner/StateReps/GoalState.h>

using namespace monolithic_pr2_planner;

GoalState::GoalState(const GoalStateParams& params):
    m_goal_state(params.goal_state), m_tolerances(4,0){
    // TODO: should i run an IK check here just to test feasibility?

    m_tolerances[Tolerances::XYZ] = params.xyz_tolerance;
    m_tolerances[Tolerances::ROLL] = params.roll_tolerance;
    m_tolerances[Tolerances::PITCH] = params.pitch_tolerance;
    m_tolerances[Tolerances::YAW] = params.yaw_tolerance;
}

bool GoalState::isGoal(GraphStatePtr graph_state) const {
    // not sure why there's a .005 here. ask ben
    ContObjectState c_tol(m_tolerances[Tolerances::XYZ]-.005, 
                          m_tolerances[Tolerances::XYZ]-.005, 
                          m_tolerances[Tolerances::XYZ]-.005,
                          m_tolerances[Tolerances::ROLL],
                          m_tolerances[Tolerances::PITCH],
                          m_tolerances[Tolerances::YAW]);
    DiscObjectState d_tol = c_tol.getDiscObjectState();
    DiscObjectState obj = graph_state->getDiscObjectState();
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
