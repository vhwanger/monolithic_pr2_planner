#pragma once
#include <monolithic_pr2_planner/StateReps/ContBaseState.h>
#include <monolithic_pr2_planner/Constants.h>
#include <monolithic_pr2_planner/OccupancyGridUser.h>
#include <pr2_collision_checker/pr2_collision_space.h>
#include <vector>
namespace monolithic_pr2_planner {
    class ContBaseState;

    class DiscBaseState : public OccupancyGridUser {
        public:
            DiscBaseState(unsigned int x, unsigned int y, 
                    unsigned int z, unsigned int theta);
            DiscBaseState(ContBaseState cont_body_state);

            bool operator==(const DiscBaseState& other);
            bool operator!=(const DiscBaseState& other);

            unsigned int getX() const { return m_state[BodyDOF::X]; };
            unsigned int getY() const { return m_state[BodyDOF::Y]; };
            unsigned int getZ() const { return m_state[BodyDOF::Z]; };
            unsigned int getTheta() const { return m_state[BodyDOF::THETA]; };

            void getValues(std::vector<unsigned int>* values);
            std::vector<unsigned int>::const_iterator getCoordBegin(){ return m_state.begin(); };
            std::vector<unsigned int>::const_iterator getCoordEnd(){ return m_state.end(); };

            void setX(unsigned int x);
            void setY(unsigned int y);
            void setZ(unsigned int z);
            void setTheta(unsigned int theta);

            ContBaseState getContBaseState();
            BodyPose getBodyPose();
        private:
            std::vector<unsigned int> m_state;
    };
}
