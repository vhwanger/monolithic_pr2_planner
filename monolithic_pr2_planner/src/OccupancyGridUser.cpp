#include <monolithic_pr2_planner/OccupancyGridUser.h>
#include <monolithic_pr2_planner/LoggerNames.h>

using namespace monolithic_pr2_planner;
using namespace boost;

boost::shared_ptr<sbpl_arm_planner::OccupancyGrid> OccupancyGridUser::m_occupancy_grid;
RobotResolutionParams OccupancyGridUser::m_resolution_params;

void OccupancyGridUser::init(OccupancyGridParams& params, RobotResolutionParams& r_params){
    m_occupancy_grid = make_shared<sbpl_arm_planner::OccupancyGrid>(
                                                          params.max_point.x, 
                                                          params.max_point.y,
                                                          params.max_point.z, 
                                                          params.env_resolution,
                                                          params.origin.x, 
                                                          params.origin.y,
                                                          params.origin.z);
    ROS_DEBUG_NAMED(INIT_LOG, "occupancy grid initialized");
    m_occupancy_grid->setReferenceFrame(params.reference_frame);
    m_resolution_params = r_params;
}
