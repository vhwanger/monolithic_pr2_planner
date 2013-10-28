#include <monolithic_pr2_planner/StateReps/DiscBaseState.h>
#include <monolithic_pr2_planner/LoggerNames.h>
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

DiscBaseState::DiscBaseState(unsigned int x, unsigned int y, unsigned int z, 
                             unsigned int theta): m_state(4){
    m_state[BodyDOF::X] = x;
    m_state[BodyDOF::Y] = y;
    m_state[BodyDOF::Z] = z;
    m_state[BodyDOF::THETA] = theta;
}

DiscBaseState::DiscBaseState(ContBaseState body_state): m_state(4){
    int x, y, z, theta;
    m_occupancy_grid->worldToGrid(body_state.getX(), 
                        body_state.getY(), 
                        body_state.getZ(),
                        x, y, z);
    double theta_res = m_resolution_params.base_theta_resolution;
    theta = static_cast<unsigned int>((normalize_angle_positive(body_state.getTheta() + theta_res*0.5))/theta_res);

    m_state[BodyDOF::X] = static_cast<unsigned int>(x);
    m_state[BodyDOF::Y] = static_cast<unsigned int>(y),
    m_state[BodyDOF::Z] = static_cast<unsigned int>(z),
    m_state[BodyDOF::THETA] = theta;
}

void DiscBaseState::getValues(vector<unsigned int>* values){
    *values = m_state;
}

ContBaseState DiscBaseState::getContBaseState(){
    return ContBaseState(*this);
}

BodyPose DiscBaseState::getBodyPose(){
    BodyPose body_pose;
    ContBaseState base_state = getContBaseState();
    body_pose.x = base_state.getX();
    body_pose.y = base_state.getY();
    body_pose.z = base_state.getZ();
    body_pose.theta = base_state.getTheta();
    return body_pose;
}
