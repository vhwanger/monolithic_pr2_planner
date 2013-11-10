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
    int vx, vy, vz, vtheta;
    m_occupancy_grid->worldToGrid(body_state.x(), 
                        body_state.y(), 
                        body_state.z(),
                        vx, vy, vz);
    double theta_res = m_resolution_params.base_theta_resolution;
    vtheta = static_cast<int>((normalize_angle_positive(body_state.theta() + theta_res*0.5))/theta_res);

    x(static_cast<int>(vx));
    y(static_cast<int>(vy));
    z(static_cast<int>(vz));
    m_state[BodyDOF::THETA] = vtheta;
}

void DiscBaseState::geStateValues(vector<int>* values) const {
    *values = m_state;
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

