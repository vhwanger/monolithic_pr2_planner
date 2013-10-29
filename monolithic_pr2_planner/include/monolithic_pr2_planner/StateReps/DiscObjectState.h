#pragma once
#include <vector>
#include <monolithic_pr2_planner/Constants.h>
#include <monolithic_pr2_planner/StateReps/ContObjectState.h>

namespace monolithic_pr2_planner {
    class ContObjectState;
    class DiscObjectState {
        public:
            bool operator==(const DiscObjectState& other);
            bool operator!=(const DiscObjectState& other);
            DiscObjectState(){};
            DiscObjectState(ContObjectState obj_state);
            DiscObjectState(unsigned int x, unsigned int y, unsigned int z, 
                            unsigned int roll, unsigned int pitch, unsigned int yaw);
            std::vector<unsigned int>::const_iterator getCoordBegin(){ return m_coord.begin(); };
            std::vector<unsigned int>::const_iterator getCoordEnd(){ return m_coord.end(); };

            unsigned int getX() const { return m_coord[ObjectPose::X]; };
            unsigned int getY() const { return m_coord[ObjectPose::Y]; };
            unsigned int getZ() const { return m_coord[ObjectPose::Z]; };
            unsigned int getRoll() const { return m_coord[ObjectPose::ROLL]; };
            unsigned int getPitch() const { return m_coord[ObjectPose::PITCH]; };
            unsigned int getYaw() const { return m_coord[ObjectPose::YAW]; };

            ContObjectState getContObjectState();

            void printToInfo(char* log_level);
            void printToDebug(char* log_level);
            
        private:
            std::vector<unsigned int> m_coord;
    };
}
