#include <monolithic_pr2_planner/StateReps/ContObjectState.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_datatypes.h>

using namespace monolithic_pr2_planner;

ContObjectState::ContObjectState(){};

ContObjectState::ContObjectState(DiscObjectState disc_obj_state):
    m_coord(6,0){
   //m_coord[ObjectPose::X] = disc_obj_state.getX(); 
   //m_coord[ObjectPose::Y] = disc_obj_state.getY(); 
   //m_coord[ObjectPose::Z] = z; 
   //m_coord[ObjectPose::ROLL] = roll;
   //m_coord[ObjectPose::PITCH] = pitch;
   //m_coord[ObjectPose::YAW] = yaw;
}

ContObjectState::ContObjectState(const geometry_msgs::PoseStamped& obj_pose):
    m_coord(6,0){
    tf::Pose pose;
    tf::poseMsgToTF(obj_pose.pose, pose);
    double roll, pitch, yaw;
    pose.getBasis().getRPY(roll, pitch, yaw);
    m_coord[ObjectPose::X] = obj_pose.pose.position.x;
    m_coord[ObjectPose::Y] = obj_pose.pose.position.y; 
    m_coord[ObjectPose::Z] = obj_pose.pose.position.z; 
    m_coord[ObjectPose::ROLL] = roll;
    m_coord[ObjectPose::PITCH] = pitch;
    m_coord[ObjectPose::YAW] = yaw;
}
