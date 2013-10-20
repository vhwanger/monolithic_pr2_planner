#include <monolithic_pr2_planner/Environment.h>

using namespace monolithic_pr2_planner;
using namespace boost;

Environment::Environment(){
    ROS_INFO("Launching monolithic_pr2_planner environment");
    m_param_catalog.fetch();

    m_arm_model = make_shared<ArmModel>(m_param_catalog.m_hardware_description_files);
    m_collision_space_mgr = make_shared<CollisionSpaceMgr>(
                m_param_catalog.m_collision_space_params, m_arm_model);
}

bool Environment::initSearchRequest(SearchRequestParamsPtr search_request_params){
    m_search_request = make_shared<SearchRequest>(search_request_params);
    return true;
}

void Environment::configureStateReps(){
    ContArmState::setRobotResolutionParams(m_param_catalog.m_robot_resolution_params);
}
