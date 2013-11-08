#pragma once
#include <bfs3d/BFS_3D.h>
#include <monolithic_pr2_planner/StateReps/DiscObjectState.h>
#include <monolithic_pr2_planner/CollisionSpaceMgr.h>
#include <monolithic_pr2_planner/OccupancyGridUser.h>
#include <memory>
#include <vector>
#include <boost/shared_ptr.hpp>

namespace monolithic_pr2_planner {
    class Heuristic : public OccupancyGridUser {
        public:
            Heuristic(CSpaceMgrPtr cspace);
            int getGoalHeuristic(GraphStatePtr state);
            // TODO cheating for now. should accept a graph state
            void setGoal(DiscObjectState& state);
        private:
            CSpaceMgrPtr m_cspace;
            std::unique_ptr<sbpl_arm_planner::BFS_3D> m_bfs;
    };
    typedef boost::shared_ptr<Heuristic> HeuristicPtr;
}
