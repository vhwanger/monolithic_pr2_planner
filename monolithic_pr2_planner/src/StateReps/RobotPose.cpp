#include <monolithic_pr2_planner/StateReps/RobotPose.h>
#include <pr2_collision_checker/pr2_collision_space.h>
#include <kdl/frames.hpp>

using namespace monolithic_pr2_planner;

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

DiscObjectState RobotPose::getMapFrameObjectState(){
    // This is an adaptation of computeContinuousObjectPose from the old
    // planner.
    // TODO: make this arm agnostic?
    std::vector<double> r_angles;

    m_right_arm.getAngles(&r_angles);
    
    // don't remember what 10 is for. ask ben.
    KDL::Frame to_wrist;
    m_right_arm.getArmModel()->computeFK(r_angles, m_base_state.getBodyPose(),
                                         10, &to_wrist);
    double roll1,pitch1,yaw1;
    to_wrist.M.GetRPY(roll1,pitch1,yaw1);
    KDL::Frame f = to_wrist * m_right_arm.getObjectOffset().Inverse();

    double wr,wp,wy;
    f.M.GetRPY(wr,wp,wy);

    return DiscObjectState(f.p.x(), f.p.y(), f.p.z(), wr, wp, wy);
}
