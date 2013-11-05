#pragma once
#include <ros/ros.h>
#include <monolithic_pr2_planner/ParameterCatalog.h>
#include <monolithic_pr2_planner/CollisionSpaceMgr.h>
#include <monolithic_pr2_planner/ArmModel.h>
#include <monolithic_pr2_planner/HashManager.h>
#include <monolithic_pr2_planner/SearchRequest.h>
#include <monolithic_pr2_planner/StateReps/GoalState.h>
#include <monolithic_pr2_planner/MotionPrimitives/MotionPrimitivesMgr.h>
#include <vector>

namespace monolithic_pr2_planner {
    class Environment {
        public:
            Environment(ros::NodeHandle nh);
            CSpaceMgrPtr getCollisionSpace(){ return m_cspace_mgr; };
            bool plan(SearchRequestParamsPtr search_request_params);
            void GetSuccs(int sourceStateID, vector<int>* succIDs, 
                          vector<int>* costs, vector<int>* actions);

        private:
            bool setStartGoal(SearchRequestPtr search_request);
            void configurePlanningDomain();
            void configureQuerySpecificParams(SearchRequestPtr search_request);

            std::vector<GoalState> m_goals;
            ParameterCatalog m_param_catalog;
            ArmModelPtr m_arm_model;
            CSpaceMgrPtr m_cspace_mgr;
            HashManager m_hash_mgr;
            MotionPrimitivesMgr m_mprims;
            ros::NodeHandle m_nodehandle;
    };
}
