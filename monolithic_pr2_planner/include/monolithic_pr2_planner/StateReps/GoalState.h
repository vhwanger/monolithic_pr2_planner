#pragma once
#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <monolithic_pr2_planner/StateReps/ContObjectState.h>
#include <vector>

namespace monolithic_pr2_planner {
    typedef struct {
        ContObjectState goal_state;
        double xyz_tolerance;
        double roll_tolerance;
        double pitch_tolerance;
        double yaw_tolerance;
        double l_free_angle;
        double r_free_angle;
    } GoalStateParams;

    // TODO: generalize this state . also have free angle defaults?
    class GoalState {
        public:
            GoalState(const GoalStateParams& params);
            bool isGoal(GraphStatePtr graph_state) const;
            unsigned int getHeuristic(GraphStatePtr graph_state);
        private:
            DiscObjectState m_goal_state;
            std::vector<double> m_tolerances;
            double l_free_angle;
            double r_free_angle;
            
    };
}
