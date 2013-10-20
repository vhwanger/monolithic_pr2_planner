#pragma once
#include <monolithic_pr2_planner/StateReps/DiscObjectState.h>

namespace monolithic_pr2_planner {
    class DiscObjectState;
    class ContObjectState {
        public:
            ContObjectState(DiscObjectState obj_state); 
            std::vector<double>::const_iterator getCoordBegin(){ return m_coord.begin(); };
            std::vector<double>::const_iterator getCoordEnd(){ return m_coord.end(); };
        private:
            std::vector<double> m_coord;
    };
}
