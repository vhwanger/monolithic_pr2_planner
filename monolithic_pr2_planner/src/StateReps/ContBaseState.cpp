#include <monolithic_pr2_planner/StateReps/ContBaseState.h>
#include <angles/angles.h>
using namespace monolithic_pr2_planner;
using namespace angles;
using namespace std;

ContBaseState::ContBaseState(){};

ContBaseState::ContBaseState(double x, double y, double z, double theta){
    setX(x); setY(y); setZ(z); setTheta(theta);
}

ContBaseState::ContBaseState(vector<double> base_pose) : 
    m_pose(base_pose){
}
ContBaseState::ContBaseState(const DiscBaseState& base_pose) :
    m_pose(4){
    m_occupancy_grid->gridToWorld(static_cast<int>(base_pose.getX()),
                                  static_cast<int>(base_pose.getY()),
                                  static_cast<int>(base_pose.getZ()),
                                  m_pose[BodyDOF::X],
                                  m_pose[BodyDOF::Y],
                                  m_pose[BodyDOF::Z]);
    double theta_res = m_resolution_params.base_theta_resolution;
    m_pose[BodyDOF::THETA] = normalize_angle_positive(static_cast<double>(base_pose.getTheta())*theta_res);
}

bool ContBaseState::interpolate(const ContBaseState& start, const ContBaseState& end,
                                int num_steps, vector<ContBaseState>* interp_steps){
    double dX = end.m_pose[BodyDOF::X] - start.m_pose[BodyDOF::X];
    double dY = end.m_pose[BodyDOF::Y] - start.m_pose[BodyDOF::Y];
    double dZ = end.m_pose[BodyDOF::Z] - start.m_pose[BodyDOF::Z];

    double dTheta = angles::shortest_angular_distance(end.m_pose[BodyDOF::THETA],
                                                      start.m_pose[BodyDOF::THETA]);
    double step_mult = 1/static_cast<double>(num_steps);
    for (int i=0; i < num_steps; i++){
        ContBaseState state(start.m_pose[BodyDOF::X] + dX * step_mult,
                            start.m_pose[BodyDOF::Y] + dY * step_mult,
                            start.m_pose[BodyDOF::Z] + dZ * step_mult,
                            start.m_pose[BodyDOF::THETA] + dTheta * step_mult);
        interp_steps->push_back(state);
    }
    return true;
}
