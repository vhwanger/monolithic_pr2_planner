#pragma once
#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <monolithic_pr2_planner/StateReps/ContObjectState.h>
#include <monolithic_pr2_planner/SearchRequest.h>
#include <monolithic_pr2_planner/Heuristic.h>
#include <vector>

namespace monolithic_pr2_planner {
    // TODO: generalize this state . also have free angle defaults?
    // TODO: implement setgoal
    class GoalState {
        public:
            GoalState(SearchRequestPtr search_request, HeuristicPtr heur);
            bool isSatisfiedBy(const GraphStatePtr& graph_state);
            unsigned int getHeuristic(GraphStatePtr graph_state);
            void storeAsSolnState(const GraphStatePtr& state){ m_full_goal_state = state; };
            bool isSolnStateID(int state_id);
            void addPotentialSolnState(const GraphStatePtr& graph_state);
            ContObjectState getContObjectState() const { return ContObjectState(m_goal_state); };

            void visualize();
        private:
            vector<int> m_possible_goals;
            DiscObjectState m_goal_state;
            GraphStatePtr m_full_goal_state;
            std::vector<double> m_tolerances;
            double l_free_angle;
            double r_free_angle;
            HeuristicPtr m_heur;
    };
}
