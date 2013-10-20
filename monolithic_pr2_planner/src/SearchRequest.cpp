#include <monolithic_pr2_planner/SearchRequest.h>
#include <monolithic_pr2_planner/StateReps/DiscObjectState.h>

using namespace monolithic_pr2_planner;

SearchRequest::SearchRequest(SearchRequestParamsPtr params){
    m_params = params;
}

RequestErrors SearchRequest::isValidSearchRequest(CSpaceMgrPtr& cspace,
                                                  ArmModelPtr& arm_model){
    RobotPose robot_pose(m_params->base_start,
                         m_params->right_arm_start,
                         m_params->left_arm_start);
    cspace->isValid(robot_pose);
    return INVALID_START;
}

