#include <monolithic_pr2_planner/SearchRequest.h>
#include <monolithic_pr2_planner/StateReps/DiscObjectState.h>

using namespace monolithic_pr2_planner;

SearchRequest::SearchRequest(SearchRequestParamsPtr params){
    m_params = params;
}

RequestErrors SearchRequest::isValid(CSpaceMgrPtr& cspace,
                                     ArmModelPtr& arm_model){
    RobotPose robot_start_pose(m_params->base_start,
                               m_params->right_arm_start,
                               m_params->left_arm_start);
    if (!cspace->isValid(robot_start_pose)){
        return INVALID_START;
    }
    return VALID_REQUEST;
}

