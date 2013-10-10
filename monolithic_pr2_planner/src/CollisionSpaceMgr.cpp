#include <monolithic_pr2_planner/CollisionSpaceMgr.h>

using namespace monolithic_pr2_planner;

CollisionSpaceMgr::CollisionSpaceMgr(CollisionSpaceParams params){
    m_grid.reset(new sbpl_arm_planner::OccupancyGrid(params.max_point.x, params.max_point.y,
                                               params.max_point.z, params.env_resolution,
                                               params.origin.x, params.origin.y,
                                               params.origin.z));
    m_grid->setReferenceFrame(params.reference_frame);
}
