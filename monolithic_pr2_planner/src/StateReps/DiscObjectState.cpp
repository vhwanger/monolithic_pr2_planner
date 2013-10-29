#include <monolithic_pr2_planner/StateReps/DiscObjectState.h>
#include <monolithic_pr2_planner/LoggerNames.h>

using namespace monolithic_pr2_planner;

bool DiscObjectState::operator==(const DiscObjectState& other){
    return m_coord == other.m_coord;
}

bool DiscObjectState::operator!=(const DiscObjectState& other){
    return !(*this == other);
}

DiscObjectState::DiscObjectState(unsigned int x, unsigned int y, 
                                 unsigned int z, unsigned int roll, 
                                 unsigned int pitch, unsigned int yaw) : m_coord(6){
   m_coord[ObjectPose::X] = x; 
   m_coord[ObjectPose::Y] = y; 
   m_coord[ObjectPose::Z] = z; 
   m_coord[ObjectPose::ROLL] = roll;
   m_coord[ObjectPose::PITCH] = pitch;
   m_coord[ObjectPose::YAW] = yaw;
}

DiscObjectState::DiscObjectState(ContObjectState obj_state): m_coord(6){
    int counter = 0;
    for (auto it=obj_state.getCoordBegin(); it != obj_state.getCoordEnd(); ++it){
        m_coord[counter] = *it;
        counter++;
    }
}

ContObjectState DiscObjectState::getContObjectState(){
    return ContObjectState(*this);
}

void DiscObjectState::printToInfo(char* log_level){
    ROS_INFO_NAMED(log_level, "object state");
    ROS_INFO_NAMED(log_level, "\t%d %d %d %d %d %d",
                   m_coord[ObjectPose::X],
                   m_coord[ObjectPose::Y],
                   m_coord[ObjectPose::Z],
                   m_coord[ObjectPose::ROLL],
                   m_coord[ObjectPose::PITCH],
                   m_coord[ObjectPose::YAW]);
}

void DiscObjectState::printToDebug(char* log_level){
    ROS_DEBUG_NAMED(log_level, "object state");
    ROS_DEBUG_NAMED(log_level, "\t%d %d %d %d %d %d",
                    m_coord[ObjectPose::X],
                    m_coord[ObjectPose::Y],
                    m_coord[ObjectPose::Z],
                    m_coord[ObjectPose::ROLL],
                    m_coord[ObjectPose::PITCH],
                    m_coord[ObjectPose::YAW]);
}
