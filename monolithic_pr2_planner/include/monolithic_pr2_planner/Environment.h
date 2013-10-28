#pragma once
#include <ros/ros.h>
#include <monolithic_pr2_planner/ParameterCatalog.h>
#include <monolithic_pr2_planner/CollisionSpaceMgr.h>
#include <monolithic_pr2_planner/ArmModel.h>
#include <monolithic_pr2_planner/HashManager.h>
#include <monolithic_pr2_planner/SearchRequest.h>

namespace monolithic_pr2_planner {
    class Environment {
        public:
            Environment(ros::NodeHandle nh);
            CSpaceMgrPtr getCollisionSpace(){ return m_collision_space_mgr; };
            bool plan(SearchRequestParamsPtr search_request_params);

        private:
            void configurePlanningDomain();
            void configureQuerySpecificParams(SearchRequestPtr search_request);

            ParameterCatalog m_param_catalog;
            ArmModelPtr m_arm_model;
            CSpaceMgrPtr m_collision_space_mgr;
            HashManager m_hash_mgr;
            ros::NodeHandle m_nodehandle;
    };
}
