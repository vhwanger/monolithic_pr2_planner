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
            DiscBaseState():m_state(4,0){};
            DiscBaseState(int x, int y, 
                    int z, int theta);
            DiscBaseState(ContBaseState cont_body_state);

            bool operator==(const DiscBaseState& other);
            bool operator!=(const DiscBaseState& other);

            int getX() const { return m_state[BodyDOF::X]; };
            int getY() const { return m_state[BodyDOF::Y]; };
            int getZ() const { return m_state[BodyDOF::Z]; };
            int getTheta() const { return m_state[BodyDOF::THETA]; };

            void getValues(std::vector<int>* values) const;
            std::vector<int>::const_iterator getCoordBegin(){ return m_state.begin(); };
            std::vector<int>::const_iterator getCoordEnd(){ return m_state.end(); };

            void setX(int x){ m_state[BodyDOF::X] = x; };
            void setY(int y){ m_state[BodyDOF::Y] = y; };
            void setZ(int z){ m_state[BodyDOF::Z] = z; };
            void setTheta(int theta){ m_state[BodyDOF::THETA] = theta; };

            ContBaseState getContBaseState() const;
            BodyPose getBodyPose() const;
        private:
            std::vector<int> m_state;
    };
}
