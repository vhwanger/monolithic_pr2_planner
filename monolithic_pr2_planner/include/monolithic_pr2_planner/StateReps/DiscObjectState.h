#pragma once
#include <vector>
#include <monolithic_pr2_planner/Constants.h>
#include <monolithic_pr2_planner/StateReps/ContObjectState.h>
#include <monolithic_pr2_planner/OccupancyGridUser.h>

namespace monolithic_pr2_planner {
    class ContObjectState;
    class DiscObjectState : public OccupancyGridUser {
        public:
            bool operator==(const DiscObjectState& other);
            bool operator!=(const DiscObjectState& other);
            DiscObjectState(){};
            DiscObjectState(ContObjectState obj_state);
            DiscObjectState(int x, int y, int z, int roll, int pitch, int yaw);
            std::vector<unsigned int>::const_iterator getCoordBegin(){ return m_coord.begin(); };
            std::vector<unsigned int>::const_iterator getCoordEnd(){ return m_coord.end(); };

            int getX() const { return m_coord[ObjectPose::X]; };
            int getY() const { return m_coord[ObjectPose::Y]; };
            int getZ() const { return m_coord[ObjectPose::Z]; };
            int getRoll() const { return m_coord[ObjectPose::ROLL]; };
            int getPitch() const { return m_coord[ObjectPose::PITCH]; };
            int getYaw() const { return m_coord[ObjectPose::YAW]; };

            void setX(int value) { m_coord[ObjectPose::X] = value; };
            void setY(int value) { m_coord[ObjectPose::Y] = value; };
            void setZ(int value) { m_coord[ObjectPose::Z] = value; };
            void setRoll(int value) { m_coord[ObjectPose::ROLL] = value; };
            void setPitch(int value) { m_coord[ObjectPose::PITCH] = value; };
            void setYaw(int value) { m_coord[ObjectPose::YAW] = value; };


            ContObjectState getContObjectState() const;

            void printToInfo(char* log_level);
            void printToDebug(char* log_level);
            
        private:
            std::vector<unsigned int> m_coord;
    };
}
