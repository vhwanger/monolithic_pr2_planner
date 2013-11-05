#pragma once
#include <monolithic_pr2_planner/StateReps/DiscBaseState.h>
#include <monolithic_pr2_planner/Constants.h>
#include <monolithic_pr2_planner/OccupancyGridUser.h>
#include <pr2_collision_checker/pr2_collision_space.h>
#include <vector>
namespace monolithic_pr2_planner {
    class DiscBaseState;
    class ContBaseState : public OccupancyGridUser {
        public:
            ContBaseState();
            ContBaseState(double x, double y, double z, double theta);
            ContBaseState(std::vector<double> base_pose);
            ContBaseState(const DiscBaseState& base_pose);

            double getX() const { return m_pose[BodyDOF::X]; };
            double getY() const { return m_pose[BodyDOF::Y]; };
            double getZ() const { return m_pose[BodyDOF::Z]; };
            double getTheta() const { return m_pose[BodyDOF::THETA]; };

            void setX(double x) { m_pose[BodyDOF::X] = x; };
            void setY(double y) { m_pose[BodyDOF::Y] = y; };
            void setZ(double z) { m_pose[BodyDOF::Z] = z; };
            void setTheta(double theta) { m_pose[BodyDOF::THETA] = normalize_angle_positive(theta); };

            DiscBaseState getDiscBaseState();
        private:
            std::vector<double> m_pose;
    };
}
