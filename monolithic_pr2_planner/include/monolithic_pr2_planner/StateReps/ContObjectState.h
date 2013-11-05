#pragma once
#include <monolithic_pr2_planner/StateReps/DiscObjectState.h>
#include <monolithic_pr2_planner/OccupancyGridUser.h>
#include <geometry_msgs/PoseStamped.h>

namespace monolithic_pr2_planner {
    class DiscObjectState;
    class ContObjectState : public OccupancyGridUser {
        public:
            // TODO: make setter functions that also normalize
            ContObjectState(); 
            ContObjectState(double x, double y, double z, double roll, double pitch, double yaw);
            ContObjectState(DiscObjectState obj_state); 
            ContObjectState(const geometry_msgs::PoseStamped& obj_pose); 
            std::vector<double>::const_iterator getCoordBegin(){ return m_coord.begin(); };
            std::vector<double>::const_iterator getCoordEnd(){ return m_coord.end(); };

            double getX() const { return m_coord[ObjectPose::X]; };
            double getY() const { return m_coord[ObjectPose::Y]; };
            double getZ() const { return m_coord[ObjectPose::Z]; };
            double getRoll() const { return m_coord[ObjectPose::ROLL]; };
            double getPitch() const { return m_coord[ObjectPose::PITCH]; };
            double getYaw() const { return m_coord[ObjectPose::YAW]; };

            DiscObjectState getDiscObjectState();

            void printToInfo(char* log_level) const;
            void printToDebug(char* log_level) const;
        private:
            std::vector<double> m_coord;
    };
}
