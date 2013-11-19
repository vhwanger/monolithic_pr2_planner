#include <monolithic_pr2_planner/StateReps/DiscBaseState.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <angles/angles.h>
using namespace monolithic_pr2_planner;
using namespace std;
using namespace angles;

bool DiscBaseState::operator==(const DiscBaseState& other) const {
    return (m_state == other.m_state);
}

bool DiscBaseState::operator!=(const DiscBaseState& other) const {
    return !(*this == other);
}

DiscBaseState::DiscBaseState(int vx, int vy, int vz,
                             int vtheta): m_state(4){
    x(vx); y(vy); z(vz); theta(vtheta);
}

DiscBaseState::DiscBaseState(ContBaseState body_state): m_state(4){
    int vx, vy, vz;
    m_occupancy_grid->worldToGrid(body_state.x(), 
                                  body_state.y(), 
                                  body_state.z(),
                                  vx, vy, vz);

    x(static_cast<int>(vx));
    y(static_cast<int>(vy));
    z(static_cast<int>(vz));
    m_state[BodyDOF::THETA] = convertContTheta(body_state.theta());
}

void DiscBaseState::geStateValues(vector<int>* values) const {
    *values = m_state;
}

int DiscBaseState::convertContDistance(double distance){
    // TODO obj_xyz and base distance discretization may not be the same! Add in
    // checks during initialization or something?
    double distance_res = m_occupancy_grid->getResolution();
    return static_cast<int>((distance + distance_res*0.5)/distance_res);
}

int DiscBaseState::convertContTheta(double theta){
    double theta_res = m_resolution_params.base_theta_resolution;
    return static_cast<int>((normalize_angle_positive(theta + theta_res*0.5))/theta_res);
}


ContBaseState DiscBaseState::getContBaseState() const {
    return ContBaseState(*this);
}

BodyPose DiscBaseState::getBodyPose() const{
    BodyPose body_pose;
    ContBaseState base_state = getContBaseState();
    body_pose.x = base_state.x();
    body_pose.y = base_state.y();
    body_pose.z = base_state.z();
    body_pose.theta = base_state.theta();
    return body_pose;
}

