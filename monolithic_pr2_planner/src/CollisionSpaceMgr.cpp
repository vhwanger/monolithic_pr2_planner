#include <monolithic_pr2_planner/CollisionSpaceMgr.h>
#include <vector>

using namespace monolithic_pr2_planner;
using namespace pr2_collision_checker;
using namespace boost;
using namespace std;

CollisionSpaceMgr::CollisionSpaceMgr(const CollisionSpaceParams& params, 
                             ArmModelPtr arm_model){
    m_grid = make_shared<sbpl_arm_planner::OccupancyGrid>(params.max_point.x, 
                                                          params.max_point.y,
                                                          params.max_point.z, 
                                                          params.env_resolution,
                                                          params.origin.x, 
                                                          params.origin.y,
                                                          params.origin.z);
    m_arm_model = arm_model;

    m_grid->setReferenceFrame(params.reference_frame);
    m_cspace = make_shared<PR2CollisionSpace>(m_arm_model->getRightArmModel(), 
                                              m_arm_model->getLeftArmModel(), 
                                              m_grid);
    ROS_INFO("Launched collision space manager");
}

void CollisionSpaceMgr::updateMap(const arm_navigation_msgs::CollisionMap& map){
    m_grid->updateFromCollisionMap(map);
}

bool CollisionSpaceMgr::isValid(RobotPose& robot_pose){
    vector<double> l_arm;
    vector<double> r_arm;
    robot_pose.getContLeftArm().getVectorOfAngles(&l_arm);
    robot_pose.getContRightArm().getVectorOfAngles(&r_arm);

    m_cspace->checkAllMotion(temp_arm1,temp_arm0,temp_body,true,dist_temp,debug_code_))
}
