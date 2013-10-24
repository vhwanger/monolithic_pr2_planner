#pragma once
#include <monolithic_pr2_planner/StateReps/ContObjectState.h>
#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <monolithic_pr2_planner/CollisionSpaceMgr.h>
#include <monolithic_pr2_planner/ArmModel.h>
#include <monolithic_pr2_planner/StateReps/RobotPose.h>
#include <kdl/frames.hpp>

namespace monolithic_pr2_planner {
    typedef struct {
        double initial_epsilon;
        double final_epsilon;
        double decrement_epsilon;
        ContObjectState obj_start_pose;
        ContObjectState obj_goal_pose;
        ContBaseState base_start;
        ContArmState left_arm_start;
        ContArmState right_arm_start;
        KDL::Frame left_arm_object;
        KDL::Frame right_arm_object;
    } SearchRequestParams;

    enum RequestErrors { 
        VALID_REQUEST,
        INVALID_START, 
        INVALID_GOAL, 
        INVALID_LEFT_ARM, 
        INVALID_RIGHT_ARM
    };

    typedef boost::shared_ptr<SearchRequestParams> SearchRequestParamsPtr;
    class SearchRequest {
        public:
            SearchRequest(SearchRequestParamsPtr params);
            RequestErrors isValid(CSpaceMgrPtr& cspace, ArmModelPtr& arm_model);
            SearchRequestParamsPtr m_params;
    };
    typedef boost::shared_ptr<SearchRequest> SearchRequestPtr;
}
