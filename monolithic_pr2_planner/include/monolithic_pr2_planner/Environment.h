#pragma once
#include <monolithic_pr2_planner/ParameterCatalog.h>
#include <monolithic_pr2_planner/CollisionSpaceMgr.h>
#include <boost/shared_ptr.hpp>

namespace monolithic_pr2_planner {
    class Environment {
        public:
            bool init();

        private:
            ParameterCatalog m_param_catalog;

            boost::shared_ptr<CollisionSpaceMgr> m_collision_space_mgr;
    };

}
