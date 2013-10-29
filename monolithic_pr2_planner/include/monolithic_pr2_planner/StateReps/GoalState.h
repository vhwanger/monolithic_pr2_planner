#pragma once
#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <monolithic_pr2_planner/StateReps/ContObjectState.h>
#include <monolithic_pr2_planner/SearchRequest.h>
#include <vector>

namespace monolithic_pr2_planner {
    // TODO: generalize this state . also have free angle defaults?
    class GoalState {
        public:
            GoalState(SearchRequestPtr search_request);
            bool isGoal(GraphStatePtr graph_state) const;
            unsigned int getHeuristic(GraphStatePtr graph_state);
            ContObjectState getContObjectState() const { return ContObjectState(m_goal_state); };
        private:
            DiscObjectState m_goal_state;
            std::vector<double> m_tolerances;
            double l_free_angle;
            double r_free_angle;
            
    };
}
