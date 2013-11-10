#include <monolithic_pr2_planner/SearchRequest.h>
#include <monolithic_pr2_planner/StateReps/DiscObjectState.h>
#include <monolithic_pr2_planner/LoggerNames.h>

using namespace monolithic_pr2_planner;

SearchRequest::SearchRequest(SearchRequestParamsPtr params){
    m_params = params;
}

bool SearchRequest::isValid(CSpaceMgrPtr& cspace){
    RobotState robot_start_pose(m_params->base_start,
                               m_params->right_arm_start,
                               m_params->left_arm_start);
    if (m_params->initial_epsilon < 1 || 
        m_params->final_epsilon < 1 || 
        m_params->decrement_epsilon < 0){
        ROS_ERROR_NAMED(INIT_LOG, "Epsilons in search request were set wrong!");
        return false;
    }
    if (!cspace->isValid(robot_start_pose)){
        ROS_ERROR_NAMED(INIT_LOG, "Starting pose is invalid.");
        return false;
    }
    return true;
}

