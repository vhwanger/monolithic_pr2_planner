#include <monolithic_pr2_planner/Environment.h>

using namespace monolithic_pr2_planner;
bool Environment::init(){
    m_param_catalog.fetch();

    m_collision_space_mgr.reset(new CollisionSpaceMgr(m_param_catalog.m_collision_space_params));
    return true;
}
