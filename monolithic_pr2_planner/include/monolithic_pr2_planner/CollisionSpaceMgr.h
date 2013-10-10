#pragma once
#include <boost/shared_ptr.hpp>
#include <monolithic_pr2_planner/ParameterCatalog.h>
#include <pr2_collision_checker/pr2_collision_space.h>

namespace monolithic_pr2_planner {
    class CollisionSpaceMgr {
        public:
            CollisionSpaceMgr(CollisionSpaceParams params);
        private:
            boost::shared_ptr<pr2_collision_checker::PR2CollisionSpace> m_cspace;
            boost::shared_ptr<sbpl_arm_planner::OccupancyGrid> m_grid;
    };
}
