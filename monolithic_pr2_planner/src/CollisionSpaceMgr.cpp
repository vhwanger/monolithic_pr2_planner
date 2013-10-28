#include <monolithic_pr2_planner/CollisionSpaceMgr.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <vector>

using namespace monolithic_pr2_planner;
using namespace pr2_collision_checker;
using namespace boost;
using namespace std;

CollisionSpaceMgr::CollisionSpaceMgr(SBPLArmModelPtr right_arm,
                                     SBPLArmModelPtr left_arm){
    m_cspace = make_shared<PR2CollisionSpace>(right_arm,
                                              left_arm,
                                              m_occupancy_grid);
    ROS_INFO_NAMED(INIT_LOG, "Launched collision space manager");
}

void CollisionSpaceMgr::updateMap(const arm_navigation_msgs::CollisionMap& map){
    m_occupancy_grid->updateFromCollisionMap(map);
}

bool CollisionSpaceMgr::isValid(RobotPose& robot_pose){
    vector<double> l_arm;
    vector<double> r_arm;
    robot_pose.getContLeftArm().getAngles(&l_arm);
    robot_pose.getContRightArm().getAngles(&r_arm);
    DiscBaseState discbody_pose = robot_pose.getDiscBaseState();
    BodyPose body_pose = robot_pose.getDiscBaseState().getBodyPose();

    double dist_temp;
    int debug_code;
    ROS_DEBUG_NAMED(CSPACE_LOG, "collision checking pose");
    // TODO make a macro for this
    ROS_DEBUG_NAMED(CSPACE_LOG, "\tbase: %f %f %f", body_pose.x, body_pose.y, 
                                                body_pose.z);
    ROS_DEBUG_NAMED(CSPACE_LOG, "\tleft arm: %f %f %f %f %f %f %f",
                    l_arm[0],
                    l_arm[1],
                    l_arm[2],
                    l_arm[3],
                    l_arm[4],
                    l_arm[5],
                    l_arm[6]);
    ROS_DEBUG_NAMED(CSPACE_LOG, "\tright arm: %f %f %f %f %f %f %f", 
                    r_arm[0],
                    r_arm[1],
                    r_arm[2],
                    r_arm[3],
                    r_arm[4],
                    r_arm[5],
                    r_arm[6]);
    return m_cspace->checkAllMotion(l_arm, r_arm, body_pose, true, dist_temp, 
                                    debug_code);
}
