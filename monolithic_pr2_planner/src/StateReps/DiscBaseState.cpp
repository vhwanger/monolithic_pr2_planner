#include <monolithic_pr2_planner/StateReps/DiscBaseState.h>
using namespace std;

DiscBaseState::getVectorOfValues(vector<unsigned int>* values){
    *values = m_state;
}
