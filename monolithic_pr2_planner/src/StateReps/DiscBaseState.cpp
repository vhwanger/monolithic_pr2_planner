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

DiscBaseState::DiscBaseState(int x, int y, int z, 
                             int theta): m_state(4){
    setX(x); setY(y); setZ(z); setTheta(theta);
}

DiscBaseState::DiscBaseState(ContBaseState body_state): m_state(4){
    int x, y, z, theta;
    m_occupancy_grid->worldToGrid(body_state.getX(), 
                        body_state.getY(), 
                        body_state.getZ(),
                        x, y, z);
    double theta_res = m_resolution_params.base_theta_resolution;
    theta = static_cast<int>((normalize_angle_positive(body_state.getTheta() + theta_res*0.5))/theta_res);

    setX(static_cast<int>(x));
    setY(static_cast<int>(y));
    setZ(static_cast<int>(z));
    m_state[BodyDOF::THETA] = theta;
}

void DiscBaseState::getValues(vector<int>* values) const {
    *values = m_state;
}

ContBaseState DiscBaseState::getContBaseState() const {
    return ContBaseState(*this);
}

BodyPose DiscBaseState::getBodyPose() const{
    BodyPose body_pose;
    ContBaseState base_state = getContBaseState();
    body_pose.x = base_state.getX();
    body_pose.y = base_state.getY();
    body_pose.z = base_state.getZ();
    body_pose.theta = base_state.getTheta();
    return body_pose;
}
