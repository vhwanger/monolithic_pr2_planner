#include <monolithic_pr2_planner/CollisionSpaceMgr.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <vector>
#include <Eigen/Core>

using namespace monolithic_pr2_planner;
using namespace pr2_collision_checker;
using namespace boost;
using namespace std;

CollisionSpaceMgr::CollisionSpaceMgr(SBPLArmModelPtr right_arm,
                                     SBPLArmModelPtr left_arm){
    m_cspace = make_shared<PR2CollisionSpace>(right_arm,
                                              left_arm,
                                              m_occupancy_grid);
    if (!m_cspace->init()){
        ROS_ERROR("cspace failed to initialize!");
    }
    ROS_INFO_NAMED(INIT_LOG, "Launched collision space manager");
}

void CollisionSpaceMgr::updateMap(const arm_navigation_msgs::CollisionMap& map){
    std::vector<Eigen::Vector3d> points;
    for (int i=0; i < (int)map.boxes.size(); i++){
        Eigen::Vector3d vect;
        vect << map.boxes[i].center.x,
        map.boxes[i].center.y,
        map.boxes[i].center.z;
        points.push_back(vect);
    }
    m_occupancy_grid->addPointsToField(points);
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
    robot_pose.printToDebug(CSPACE_LOG);
    return m_cspace->checkAllMotion(l_arm, r_arm, body_pose, true, dist_temp, 
                                    debug_code);
}

bool CollisionSpaceMgr::isValidMotion(const GraphState& source_state, 
                                      const MotionPrimitivePtr& mprim,
                                      unique_ptr<GraphState>& successor){
    successor = mprim->apply(source_state);
    ROS_DEBUG_NAMED(SEARCH_LOG, "source state:");
    source_state.printToDebug(SEARCH_LOG);
    ROS_DEBUG_NAMED(SEARCH_LOG, "successor state:");
    successor->printToDebug(SEARCH_LOG);



    return false;
}
