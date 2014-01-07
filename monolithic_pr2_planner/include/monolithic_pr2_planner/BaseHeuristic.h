#pragma once
#include <sbpl/headers.h>
#include <monolithic_pr2_planner/StateReps/DiscObjectState.h>
#include <monolithic_pr2_planner/OccupancyGridUser.h>
#include <boost/shared_ptr.hpp>
#include <memory>

namespace monolithic_pr2_planner {
    class BaseHeuristic : public OccupancyGridUser{
        public:
            BaseHeuristic();
            ~BaseHeuristic();
            void setGoal(DiscObjectState& state);
            void loadMap(std::vector<signed char>& data);
        private:
            std::unique_ptr<sbpl_bfs_2d> m_bfs;
            unsigned int m_size_col;
            unsigned int m_size_row;
            int** m_grid;
    };
    typedef boost::shared_ptr<BaseHeuristic> BaseHeuristicPtr;
}
