#include <monolithic_pr2_planner/StateReps/ContBaseState.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <angles/angles.h>
using namespace monolithic_pr2_planner;
using namespace angles;
using namespace std;

ContBaseState::ContBaseState(){};

ContBaseState::ContBaseState(double vx, double vy, double vz, double vtheta):
    m_pose(4){
    x(vx); y(vy); z(vz); theta(vtheta);
}

ContBaseState::ContBaseState(vector<double> base_pose) : 
    m_pose(base_pose){
}
ContBaseState::ContBaseState(const DiscBaseState& base_pose) :
    m_pose(4){
    m_occupancy_grid->gridToWorld(base_pose.x(),
                                  base_pose.y(),
                                  base_pose.z(),
                                  m_pose[BodyDOF::X],
                                  m_pose[BodyDOF::Y],
                                  m_pose[BodyDOF::Z]);
    double theta_res = m_resolution_params.base_theta_resolution;
    m_pose[BodyDOF::THETA] = normalize_angle_positive(static_cast<double>(base_pose.theta())*theta_res);
}

vector<ContBaseState> ContBaseState::interpolate(const ContBaseState& start, 
                                                 const ContBaseState& end,
                                                 int num_steps){
    vector<ContBaseState> interp_steps;
    double dX = end.m_pose[BodyDOF::X] - start.m_pose[BodyDOF::X];
    double dY = end.m_pose[BodyDOF::Y] - start.m_pose[BodyDOF::Y];
    double dZ = end.m_pose[BodyDOF::Z] - start.m_pose[BodyDOF::Z];

    double dTheta = angles::shortest_angular_distance(start.m_pose[BodyDOF::THETA],
                                                      end.m_pose[BodyDOF::THETA]);

    double step_mult = 1/static_cast<double>(num_steps);
    for (int i=0; i <= num_steps; i++){
        ContBaseState state(start.m_pose[BodyDOF::X] + i*dX*step_mult,
                            start.m_pose[BodyDOF::Y] + i*dY*step_mult,
                            start.m_pose[BodyDOF::Z] + i*dZ*step_mult,
                            start.m_pose[BodyDOF::THETA] + i*dTheta*step_mult);
        interp_steps.push_back(state);
    }
    return interp_steps;
}

void ContBaseState::printToDebug(char* logger){
    ROS_DEBUG_NAMED(logger, "%f %f %f %f",
                    m_pose[BodyDOF::X],
                    m_pose[BodyDOF::Y],
                    m_pose[BodyDOF::Z],
                    m_pose[BodyDOF::THETA]);
}
