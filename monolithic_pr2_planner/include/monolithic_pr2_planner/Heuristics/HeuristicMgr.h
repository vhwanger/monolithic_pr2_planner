#pragma once
#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <monolithic_pr2_planner/OccupancyGridUser.h>
#include <monolithic_pr2_planner/StateReps/GoalState.h>
#include <monolithic_pr2_planner/Heuristics/AbstractHeuristic.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <memory>
#include <vector>
#include <boost/shared_ptr.hpp>

namespace monolithic_pr2_planner {
    /*! \brief The manager class that handles all the heuristics.
     */
    class HeuristicMgr {
        public:
            // Add methods for all possible kinds of heuristics. Whenever a new
            // heuristic type is added, a corresponding add<type>Heur() method
            // needs to be added here. Returns the id of the heuristic in the
            // internal m_heuristics vector.
            int add3DHeur();
            int add2DHeur();

            // Updates the collision map for the heuristics that need them.
            // Doesn't take in an argument because each 3D heuristic shares the
            // occupancy grid singleton.
            void update3DHeuristicMaps();

            // Updates the 2D map for the heuristics that need them
            void update2DHeuristicMaps(const std::vector<signed char>& data);

            // TODO: Multiple goals should just take the goal state and the heuristic ID.
            void setGoal(GoalState& state);

            // Get the heuristic value
            std::vector<int> getGoalHeuristic(const GraphStatePtr& state);
        private:
            std::vector<AbstractHeuristicPtr> m_heuristics;
    };
    typedef boost::shared_ptr<HeuristicMgr> HeuristicMgrPtr;
}
