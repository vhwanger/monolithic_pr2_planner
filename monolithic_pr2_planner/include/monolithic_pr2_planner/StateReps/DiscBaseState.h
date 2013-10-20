#pragma once
#include <monolithic_pr2_planner/StateReps/ContBaseState.h>
#include <monolithic_pr2_planner/Constants.h>
#include <vector>
namespace monolithic_pr2_planner {
    class ContBaseState;

    class DiscBaseState {
        public:
            DiscBaseState(unsigned int x, unsigned int y, 
                             unsigned int z, unsigned int theta);
            DiscBaseState(ContBaseState cont_body_state);

            unsigned int getBodyX() const { return m_state[BodyDOF::X]; };
            unsigned int getBodyY() const { return m_state[BodyDOF::Y]; };
            unsigned int getBodyZ() const { return m_state[BodyDOF::Z]; };
            unsigned int getBodyTheta() const { return m_state[BodyDOF::THETA]; };

            void getVectorOfValues(std::vector<unsigned int>* values);
            std::vector<unsigned int>::const_iterator getCoordBegin(){ return m_state.begin(); };
            std::vector<unsigned int>::const_iterator getCoordEnd(){ return m_state.end(); };

            void setBodyX(unsigned int x);
            void setBodyY(unsigned int y);
            void setBodyZ(unsigned int z);
            void setBodyTheta(unsigned int theta);

            ContBaseState getContBaseState();
        private:
            std::vector<unsigned int> m_state;
    };
}
