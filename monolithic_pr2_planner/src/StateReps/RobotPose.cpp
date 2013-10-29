#include <monolithic_pr2_planner/StateReps/RobotPose.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <monolithic_pr2_planner/Constants.h>
#include <pr2_collision_checker/pr2_collision_space.h>
#include <kdl/frames.hpp>
#include <vector>

using namespace monolithic_pr2_planner;

boost::shared_ptr<PViz> RobotPose::m_pviz;

bool RobotPose::operator==(const RobotPose& other){
    return (m_base_state == other.m_base_state &&
            m_right_arm == other.m_right_arm &&
            m_left_arm == other.m_left_arm);
}

bool RobotPose::operator!=(const RobotPose& other){
    return !(*this == other);
}

RobotPose::RobotPose(ContBaseState base_state, RightContArmState r_arm, 
                     LeftContArmState l_arm):
    m_base_state(base_state), m_right_arm(r_arm), m_left_arm(l_arm){
}

ContBaseState RobotPose::getContBaseState(){
    return ContBaseState(m_base_state);    
}

void RobotPose::printToDebug(char* log_level){
    ContBaseState base_state = m_base_state.getContBaseState();
    ROS_DEBUG_NAMED(log_level, "\tbase: %f %f %f", 
                   base_state.getX(),
                   base_state.getY(),
                   base_state.getZ());
    std::vector<double> l_arm, r_arm;
    m_right_arm.getAngles(&r_arm);
    m_left_arm.getAngles(&l_arm);
    ROS_DEBUG_NAMED(log_level, "\tleft arm: %f %f %f %f %f %f %f",
                    l_arm[Joints::SHOULDER_PAN],
                    l_arm[Joints::SHOULDER_LIFT],
                    l_arm[Joints::UPPER_ARM_ROLL],
                    l_arm[Joints::ELBOW_FLEX],
                    l_arm[Joints::FOREARM_ROLL],
                    l_arm[Joints::WRIST_FLEX],
                    l_arm[Joints::WRIST_ROLL]);
    ROS_DEBUG_NAMED(log_level, "\tright arm: %f %f %f %f %f %f %f", 
                    r_arm[Joints::SHOULDER_PAN],
                    r_arm[Joints::SHOULDER_LIFT],
                    r_arm[Joints::UPPER_ARM_ROLL],
                    r_arm[Joints::ELBOW_FLEX],
                    r_arm[Joints::FOREARM_ROLL],
                    r_arm[Joints::WRIST_FLEX],
                    r_arm[Joints::WRIST_ROLL]);
}

void RobotPose::printToInfo(char* log_level){
    ContBaseState base_state = m_base_state.getContBaseState();
    ROS_INFO_NAMED(log_level, "\tbase: %f %f %f", 
                   base_state.getX(),
                   base_state.getY(),
                   base_state.getZ());
    std::vector<double> l_arm, r_arm;
    m_right_arm.getAngles(&r_arm);
    m_left_arm.getAngles(&l_arm);
    ROS_INFO_NAMED(log_level, "\tleft arm: %f %f %f %f %f %f %f",
                    l_arm[0],
                    l_arm[1],
                    l_arm[2],
                    l_arm[3],
                    l_arm[4],
                    l_arm[5],
                    l_arm[6]);
    ROS_INFO_NAMED(log_level, "\tright arm: %f %f %f %f %f %f %f", 
                    r_arm[0],
                    r_arm[1],
                    r_arm[2],
                    r_arm[3],
                    r_arm[4],
                    r_arm[5],
                    r_arm[6]);
}

void RobotPose::setPViz(boost::shared_ptr<PViz> pviz){
    m_pviz = pviz;
}

void RobotPose::visualize(){
    std::vector<double> l_arm, r_arm;
    m_left_arm.getAngles(&l_arm);
    m_right_arm.getAngles(&r_arm);
    BodyPose body_pose = m_base_state.getBodyPose();
    m_pviz->visualizeRobot(r_arm, l_arm, body_pose, 150, std::string("planner"), 0);
}

ContObjectState RobotPose::getDiscMapFrameObjectState(){
    // This is an adaptation of computeContinuousObjectPose from the old
    // planner.  TODO: make this arm agnostic?
    std::vector<double> r_angles;
    m_right_arm.getAngles(&r_angles);
    SBPLArmModelPtr arm_model = m_right_arm.getArmModel();

    // don't remember what 10 is for. ask ben.
    KDL::Frame to_wrist;
    arm_model->computeFK(r_angles, m_base_state.getBodyPose(), 10, &to_wrist);
    double roll1,pitch1,yaw1;
    to_wrist.M.GetRPY(roll1,pitch1,yaw1);
    KDL::Frame f = to_wrist * m_right_arm.getObjectOffset().Inverse();

    double wr,wp,wy;
    f.M.GetRPY(wr,wp,wy);

    return ContObjectState(f.p.x(), f.p.y(), f.p.z(), wr, wp, wy);
}

