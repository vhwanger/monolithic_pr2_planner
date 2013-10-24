#include <monolithic_pr2_planner/Environment.h>
#include <monolithic_pr2_planner/OccupancyGridUser.h>
#include <monolithic_pr2_planner/StateReps/RobotPose.h>
#include <monolithic_pr2_planner/StateReps/ContArmState.h>

using namespace monolithic_pr2_planner;
using namespace boost;

Environment::Environment(){
    ROS_INFO("Launching monolithic_pr2_planner environment");
    m_param_catalog.fetch();

    // since so many objects are using occupancy grid, I'm making a singleton
    // for everybody to share.
    OccupancyGridUser::init(m_param_catalog.m_occupancy_grid_params,
                            m_param_catalog.m_robot_resolution_params);

    // need to initialize left and right arm models

    m_collision_space_mgr = make_shared<CollisionSpaceMgr>(m_arm_model);
}

bool Environment::plan(SearchRequestParamsPtr search_request_params){
    m_search_request = make_shared<SearchRequest>(search_request_params);
    m_search_request->isValid(m_collision_space_mgr, m_arm_model);

    RobotPose start_pose(m_search_request->m_params->base_start, 
                         m_search_request->m_params->right_arm_start,
                         m_search_request->m_params->left_arm_start);
    
    //m_hash_mgr.save();

    return true;
}

void Environment::configureStateReps(){
    ContArmState::setRobotResolutionParams(m_param_catalog.m_robot_resolution_params);
    LeftContArmState::setArmModel(m_param_catalog.m_hardware_description_files);
    RightContArmState::setArmModel(m_param_catalog.m_hardware_description_files);
}
