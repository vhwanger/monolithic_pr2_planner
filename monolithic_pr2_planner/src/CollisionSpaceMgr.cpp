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

bool CollisionSpaceMgr::isValidMotion(const GraphState& source_state, 
                                      const MotionPrimitivePtr& mprim,
                                      GraphStatePtr& successor){
    // couldn't generate a graph state for the successor
    if (!mprim->apply(source_state, successor)){
        return false;
    }

    // now let's check the validity of the new graph state
    int motion_type = mprim->motion_type();
    if (motion_type == MPrim_Types::BASE){
        if (!isValidAfterBaseMotion(successor, mprim)){
            ROS_DEBUG_NAMED(SEARCH_LOG, "invalid after base motion");
            return false;
        }
    } else if (motion_type == MPrim_Types::ARM){
        if (!isValidAfterArmMotion(successor, mprim)) return false;
    } else {
        throw std::invalid_argument("not a valid motion primitive type");
    }

    // let's check the validity of all intermediate poses
    if (motion_type == MPrim_Types::BASE){
        if (!isBaseIntermStatesValid(source_state, mprim)) return false;
    } else if (motion_type == MPrim_Types::ARM){
        //if (!isArmsIntermStatesValid(source_state, mprim)) return false;
    } else {
        throw std::invalid_argument("not a valid motion primitive type");
    }
    return true;
}

bool CollisionSpaceMgr::isValidAfterArmMotion(GraphStatePtr& successor,
                                              const MotionPrimitivePtr& mprim) const {
    RobotState pose = successor->robot_pose();
    vector<double> r_arm(7), l_arm(7);
    pose.right_arm().getAngles(&r_arm);
    pose.left_arm().getAngles(&l_arm);
    BodyPose body_pose = pose.base_state().getBodyPose();
    bool verbose = true;
    double dist;
    int debug;

    return m_cspace->checkArmsMotion(l_arm, r_arm, body_pose, verbose, dist, debug);
}

bool CollisionSpaceMgr::isValidAfterBaseMotion(GraphStatePtr& successor,
                                               const MotionPrimitivePtr& mprim) const {
    RobotState pose = successor->robot_pose();
    vector<double> r_arm(7), l_arm(7);
    pose.right_arm().getAngles(&r_arm);
    pose.left_arm().getAngles(&l_arm);
    BodyPose body_pose = pose.base_state().getBodyPose();
    bool verbose = true;
    double dist;
    int debug;

    return m_cspace->checkBaseMotion(l_arm, r_arm, body_pose, verbose, dist, debug);
}

bool CollisionSpaceMgr::isBaseIntermStatesValid(const GraphState& source_state,
                                                const MotionPrimitivePtr& mprim){

    RobotState pose = source_state.robot_pose();
    vector<double> r_arm(7), l_arm(7);
    pose.right_arm().getAngles(&r_arm);
    pose.left_arm().getAngles(&l_arm);
    BodyPose body_pose = pose.base_state().getBodyPose();
    bool verbose = true;
    double dist;
    int debug;

    // TODO make sure this skips the first and last points in the intermediate
    // steps list - they are repeats of the start and end position
    BOOST_FOREACH(auto interm_steps, mprim->getIntermSteps()){
        // TODO NOOOOOOO. Move this somewhere else!~
        BodyPose body_pose_interm = body_pose;

        body_pose_interm.x += interm_steps[GraphStateElement::BASE_X];
        body_pose_interm.y += interm_steps[GraphStateElement::BASE_Y];
        body_pose_interm.theta += interm_steps[GraphStateElement::BASE_THETA];
        if (!m_cspace->checkBaseMotion(l_arm, r_arm, body_pose_interm, verbose, dist, debug))
            return false;
    }
    return true;
}
//bool isArmsIntermStatesValid(const GraphState& source_state,
                             //const MotionPrimitivePtr& mprim){
    // TODO - because the arms currently have no intermediate points, i'm not
    // going to implement this right now.
//}


