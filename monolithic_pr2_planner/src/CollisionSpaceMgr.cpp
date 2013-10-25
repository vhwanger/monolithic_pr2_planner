#include <monolithic_pr2_planner/CollisionSpaceMgr.h>
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
    ROS_INFO("Launched collision space manager");
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

    return m_cspace->checkAllMotion(l_arm, r_arm, body_pose, true, dist_temp, debug_code);
}
