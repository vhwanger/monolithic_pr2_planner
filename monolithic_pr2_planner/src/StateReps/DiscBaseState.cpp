#include <monolithic_pr2_planner/StateReps/DiscBaseState.h>
#include <angles/angles.h>
using namespace monolithic_pr2_planner;
using namespace std;
using namespace angles;

bool DiscBaseState::operator==(const DiscBaseState& other){
    return (m_state == other.m_state);
}

bool DiscBaseState::operator!=(const DiscBaseState& other){
    return !(*this == other);
}

DiscBaseState::DiscBaseState(unsigned int x, unsigned int y, unsigned int z, unsigned int theta){
    m_state[BodyDOF::X] = x;
    m_state[BodyDOF::Y] = y;
    m_state[BodyDOF::Z] = z;
    m_state[BodyDOF::THETA] = theta;
}
DiscBaseState::DiscBaseState(ContBaseState body_state){
    int x, y, z, theta;
    m_occupancy_grid->worldToGrid(body_state.getX(), 
                        body_state.getY(), 
                        body_state.getZ(),
                        x, y, z);
    double theta_res = m_resolution_params.base_theta_resolution;
    theta = (int)((normalize_angle_positive(body_state.getTheta() + theta_res*0.5))/theta_res);

    DiscBaseState(static_cast<unsigned int>(x),
                  static_cast<unsigned int>(y),
                  static_cast<unsigned int>(z),
                  static_cast<unsigned int>(theta));
}

void DiscBaseState::getVectorOfValues(vector<unsigned int>* values){
    *values = m_state;
}
