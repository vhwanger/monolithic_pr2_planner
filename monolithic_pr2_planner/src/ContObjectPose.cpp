#include <monolithic_pr2_planner/StateReps/ContObjectState.h>

using namespace monolithic_pr2_planner;

ContObjectState::ContObjectState(DiscObjectState disc_obj_state){
   m_coord[ObjectPose::X] = x; 
   m_coord[ObjectPose::Y] = y; 
   m_coord[ObjectPose::Z] = z; 
   m_coord[ObjectPose::ROLL] = roll;
   m_coord[ObjectPose::PITCH] = pitch;
   m_coord[ObjectPose::YAW] = yaw;
}
