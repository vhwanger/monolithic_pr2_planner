#pragma once
#include <monolithic_pr2_planner/StateReps/DiscObjectState.h>
#include <monolithic_pr2_planner/OccupancyGridUser.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>

namespace monolithic_pr2_planner {
    class DiscObjectState;
    class ContObjectState : public OccupancyGridUser {
        public:
            // TODO: make setter functions that also normalize
            ContObjectState(); 
            ContObjectState(double x, double y, double z, double roll, double pitch, double yaw);
            ContObjectState(DiscObjectState obj_state); 
            ContObjectState(const geometry_msgs::PoseStamped& obj_pose); 

            void getStateValues(std::vector<double>* values){ *values = m_coord; };

            double x() const { return m_coord[ObjectPose::X]; };
            double y() const { return m_coord[ObjectPose::Y]; };
            double z() const { return m_coord[ObjectPose::Z]; };
            double roll() const { return m_coord[ObjectPose::ROLL]; };
            double pitch() const { return m_coord[ObjectPose::PITCH]; };
            double yaw() const { return m_coord[ObjectPose::YAW]; };

            void x(double value) { m_coord[ObjectPose::X] = value; };
            void y(double value) { m_coord[ObjectPose::Y] = value; };
            void z(double value) { m_coord[ObjectPose::Z] = value; };
            void roll(double value) { m_coord[ObjectPose::ROLL] = normalize(value); };
            void pitch(double value) { m_coord[ObjectPose::PITCH] = normalize(value); };
            void yaw(double value) { m_coord[ObjectPose::YAW] = normalize(value); };


            DiscObjectState getDiscObjectState();

            void printToInfo(char* log_level) const;
            void printToDebug(char* log_level) const;
        private:
            inline double normalize(double value){
                return normalize_angle_positive(value);
            }
            std::vector<double> m_coord;
    };
}
