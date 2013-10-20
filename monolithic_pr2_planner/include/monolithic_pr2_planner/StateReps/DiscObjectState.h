#pragma once
#include <vector>
#include <monolithic_pr2_planner/Constants.h>
#include <monolithic_pr2_planner/StateReps/ContObjectState.h>

namespace monolithic_pr2_planner {
    class ContObjectState;
    class DiscObjectState {
        public:
            DiscObjectState(ContObjectState obj_state);
            DiscObjectState(unsigned int x, unsigned int y, unsigned int z, 
                            unsigned int roll, unsigned int pitch, unsigned int yaw);
            std::vector<unsigned int>::const_iterator getCoordBegin(){ return m_coord.begin(); };
            std::vector<unsigned int>::const_iterator getCoordEnd(){ return m_coord.end(); };
            
        private:
            std::vector<unsigned int> m_coord;
    };
}
