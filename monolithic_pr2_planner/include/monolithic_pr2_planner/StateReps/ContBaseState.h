#pragma once
#include <monolithic_pr2_planner/StateReps/DiscBaseState.h>
#include <monolithic_pr2_planner/Constants.h>
#include <monolithic_pr2_planner/OccupancyGridUser.h>
#include <vector>
namespace monolithic_pr2_planner {
    class DiscBaseState;
    class ContBaseState : public OccupancyGridUser {
        public:
            ContBaseState();
            ContBaseState(double x, double y, double z, double theta);
            ContBaseState(std::vector<double> base_pose);
            ContBaseState(DiscBaseState base_pose);

            double getX() const { return m_pose[BodyDOF::X]; };
            double getY() const { return m_pose[BodyDOF::Y]; };
            double getZ() const { return m_pose[BodyDOF::Z]; };
            double getTheta() const { return m_pose[BodyDOF::THETA]; };

            void setX(double x);
            void setY(double y);
            void setZ(double z);
            void setTheta(double theta);

            DiscBaseState getDiscBaseState();
        private:
            std::vector<double> m_pose;
    };
}
