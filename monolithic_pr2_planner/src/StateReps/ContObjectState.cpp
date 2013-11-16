#include <monolithic_pr2_planner/StateReps/ContObjectState.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>

using namespace monolithic_pr2_planner;
using namespace angles;

ContObjectState::ContObjectState(){};
ContObjectState::ContObjectState(double vx, double vy, double vz,
                                 double vroll, double vpitch, double vyaw):
    m_coord(6,0){
    m_coord[ObjectPose::X] = vx;
    m_coord[ObjectPose::Y] = vy;
    m_coord[ObjectPose::Z] = vz;
    roll(vroll);
    pitch(vpitch);
    yaw(vyaw);
}

ContObjectState::ContObjectState(DiscObjectState obj_state):
    m_coord(6,0){
        m_occupancy_grid->gridToWorld(obj_state.x(),
                                      obj_state.y(),
                                      obj_state.z(),
                                      m_coord[ObjectPose::X],
                                      m_coord[ObjectPose::Y],
                                      m_coord[ObjectPose::Z]);
        double rpy_res = m_resolution_params.obj_rpy_resolution;
        roll(obj_state.roll()*rpy_res);
        pitch(obj_state.pitch()*rpy_res);
        yaw(obj_state.yaw()*rpy_res);
}

ContObjectState::ContObjectState(const geometry_msgs::PoseStamped& obj_pose):
    m_coord(6,0){
    tf::Pose pose;
    tf::poseMsgToTF(obj_pose.pose, pose);
    double vroll, vpitch, vyaw;
    pose.getBasis().getRPY(vroll, vpitch, vyaw);
    m_coord[ObjectPose::X] = obj_pose.pose.position.x;
    m_coord[ObjectPose::Y] = obj_pose.pose.position.y; 
    m_coord[ObjectPose::Z] = obj_pose.pose.position.z; 
    roll(vroll);
    pitch(vpitch);
    yaw(vyaw);
}

DiscObjectState ContObjectState::getDiscObjectState(){
    return ContObjectState(*this); 
}

void ContObjectState::printToInfo(char* log_level) const {
    ROS_INFO_NAMED(log_level, "object state");
    ROS_INFO_NAMED(log_level, "\t%f %f %f %f %f %f",
                   m_coord[ObjectPose::X],
                   m_coord[ObjectPose::Y],
                   m_coord[ObjectPose::Z],
                   m_coord[ObjectPose::ROLL],
                   m_coord[ObjectPose::PITCH],
                   m_coord[ObjectPose::YAW]);
}

void ContObjectState::printToDebug(char* log_level) const {
    ROS_DEBUG_NAMED(log_level, "object state");
    ROS_DEBUG_NAMED(log_level, "\t%f %f %f %f %f %f",
                    m_coord[ObjectPose::X],
                    m_coord[ObjectPose::Y],
                    m_coord[ObjectPose::Z],
                    m_coord[ObjectPose::ROLL],
                    m_coord[ObjectPose::PITCH],
                    m_coord[ObjectPose::YAW]);
}
