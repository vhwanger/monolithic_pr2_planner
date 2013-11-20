#include <monolithic_pr2_planner/CollisionSpaceMgr.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <monolithic_pr2_planner/Constants.h>
#include <boost/foreach.hpp>
#include <stdexcept>
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

bool CollisionSpaceMgr::isValid(RobotState& robot_pose){
    vector<double> l_arm;
    vector<double> r_arm;
    robot_pose.left_arm().getAngles(&l_arm);
    robot_pose.right_arm().getAngles(&r_arm);
    DiscBaseState discbody_pose = robot_pose.base_state();
    BodyPose body_pose = robot_pose.base_state().getBodyPose();

    double dist_temp;
    int debug_code;
    ROS_DEBUG_NAMED(CSPACE_LOG, "collision checking pose");
    ROS_DEBUG_NAMED(CSPACE_LOG, "body pose is %f %f %f", body_pose.x, body_pose.y, body_pose.z);
    robot_pose.printToDebug(CSPACE_LOG);
    return m_cspace->checkAllMotion(l_arm, r_arm, body_pose, true, dist_temp, 
                                    debug_code);
}

// TODO bounds check spine, collision check spine motion
bool CollisionSpaceMgr::isValidSuccessor(const GraphState& successor,
                                         const TransitionData& t_data){
    RobotState pose = successor.robot_pose();
    vector<double> r_arm(7), l_arm(7);
    pose.right_arm().getAngles(&r_arm);
    pose.left_arm().getAngles(&l_arm);
    BodyPose body_pose = pose.base_state().getBodyPose();
    bool verbose = false;
    double dist;
    int debug;

    bool onlyBaseMotion = (t_data.motion_type() == MPrim_Types::BASE ||
                           t_data.motion_type() == MPrim_Types::BASE_ADAPTIVE);
    bool onlyArmMotion = (t_data.motion_type() == MPrim_Types::ARM ||
                          t_data.motion_type() == MPrim_Types::ARM_ADAPTIVE);
    if (onlyBaseMotion){
        return m_cspace->checkBaseMotion(l_arm, r_arm, body_pose, verbose, dist, debug);
    } else if (onlyArmMotion){
        return m_cspace->checkArmsMotion(l_arm, r_arm, body_pose, verbose, dist, debug);
    } else if (t_data.motion_type() == MPrim_Types::TORSO){
        return m_cspace->checkSpineMotion(l_arm, r_arm, body_pose, verbose, dist, debug);
    } else {
        throw std::invalid_argument("not a valid motion primitive type");
    }

    return true;
}

// TODO need to fix this to collision check the right data in t_data
bool CollisionSpaceMgr::isValidTransitionStates(const TransitionData& t_data){
    for (auto robot_state : t_data.interm_robot_steps()){
        vector<double> r_arm(7), l_arm(7);
        robot_state.right_arm().getAngles(&r_arm);
        robot_state.left_arm().getAngles(&l_arm);
        BodyPose body_pose = robot_state.base_state().getBodyPose();
        bool verbose = false;
        double dist;
        int debug;
    
        // let's check the validity of all intermediate poses
        bool onlyBaseMotion = (t_data.motion_type() == MPrim_Types::BASE ||
                               t_data.motion_type() == MPrim_Types::BASE_ADAPTIVE);
        bool onlyArmMotion = (t_data.motion_type() == MPrim_Types::ARM ||
                              t_data.motion_type() == MPrim_Types::ARM_ADAPTIVE);
        if (onlyBaseMotion){
            return m_cspace->checkBaseMotion(l_arm, r_arm, body_pose, verbose, dist, debug);
        } else if (onlyArmMotion){
            ROS_DEBUG_NAMED(CSPACE_LOG, "skipping the intermediate points for arms because there are none.");
        } else {
            throw std::invalid_argument("not a valid motion primitive type");
        }
    }
    return true;
}

