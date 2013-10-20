#pragma once
#include <monolithic_pr2_planner/ParameterCatalog.h>
#include <monolithic_pr2_planner/CollisionSpaceMgr.h>
#include <monolithic_pr2_planner/ArmModel.h>
#include <monolithic_pr2_planner/HashManager.h>
#include <monolithic_pr2_planner/SearchRequest.h>
#include <boost/shared_ptr.hpp>

namespace monolithic_pr2_planner {
    class Environment {
        public:
            Environment();
            CSpaceMgrPtr getCollisionSpace(){ return m_collision_space_mgr; };
            bool initSearchRequest(SearchRequestParamsPtr search_request_params);

        private:
            void configureStateReps();
            ParameterCatalog m_param_catalog;
            ArmModelPtr m_arm_model;
            CSpaceMgrPtr m_collision_space_mgr;
            SearchRequestPtr m_search_request;
            HashManager m_hash_mgr;
    };
}
