#include <monolithic_pr2_planner/CollisionSpaceMgr.h>
#include <vector>

using namespace monolithic_pr2_planner;
using namespace pr2_collision_checker;
using namespace boost;
using namespace std;

CollisionSpaceMgr::CollisionSpaceMgr(ArmModelPtr arm_model){
    m_arm_model = arm_model;

    m_cspace = make_shared<PR2CollisionSpace>(m_arm_model->getRightArmModel(), 
                                              m_arm_model->getLeftArmModel(), 
                                              m_occupancy_grid);
    ROS_INFO("Launched collision space manager");
}

void CollisionSpaceMgr::updateMap(const arm_navigation_msgs::CollisionMap& map){
    m_occupancy_grid->updateFromCollisionMap(map);
}

bool CollisionSpaceMgr::isValid(RobotPose& robot_pose){
    vector<double> l_arm;
    vector<double> r_arm;
    robot_pose.getContLeftArm().getVectorOfAngles(&l_arm);
    robot_pose.getContRightArm().getVectorOfAngles(&r_arm);
    DiscBaseState base_state = robot_pose.getContBaseState();

    // yucky code to make things work with pr2_collision_checker
    BodyPose body_pose;
    body_pose.x = base_state.getX();
    body_pose.y = base_state.getY();
    body_pose.z = base_state.getZ();
    body_pose.theta = base_state.getTheta();
    double dist_temp;
    int debug_code;

    return m_cspace->checkAllMotion(l_arm, r_arm, body_pose, true, dist_temp, debug_code);
}
