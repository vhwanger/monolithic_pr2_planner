#include <monolithic_pr2_planner/StateReps/ContBaseState.h>
#include <angles/angles.h>
using namespace monolithic_pr2_planner;
using namespace angles;
using namespace std;

ContBaseState::ContBaseState(){};

ContBaseState::ContBaseState(vector<double> base_pose) : 
    m_pose(base_pose){
}
ContBaseState::ContBaseState(DiscBaseState base_pose){
    m_occupancy_grid->gridToWorld(base_pose.getX(),
                                  base_pose.getY(),
                                  base_pose.getZ(),
                                  m_pose[BodyDOF::X],
                                  m_pose[BodyDOF::Y],
                                  m_pose[BodyDOF::Z]);
    double theta_res = m_resolution_params.base_theta_resolution;
    m_pose[BodyDOF::THETA] = normalize_angle_positive(double(base_pose.getTheta())*theta_res);
}