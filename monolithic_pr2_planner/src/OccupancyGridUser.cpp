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
    ROS_INFO_NAMED(INIT_LOG, "occupancy grid initialized");
    ROS_DEBUG_NAMED(INIT_LOG, "\tsize: %f %f %f ",
                          params.max_point.x, 
                          params.max_point.y,
                          params.max_point.z);
    ROS_DEBUG_NAMED(INIT_LOG, "\tresolution: %f ",
                          params.env_resolution);
    ROS_DEBUG_NAMED(INIT_LOG, "\torigin: %f %f %f",  
                          params.origin.x, 
                          params.origin.y,
                          params.origin.z);
    m_occupancy_grid->setReferenceFrame(params.reference_frame);
    m_resolution_params = r_params;
}
