#pragma once
#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <monolithic_pr2_planner/StateReps/ContBaseState.h>
#include <monolithic_pr2_planner/StateReps/DiscBaseState.h>
#include <geometry_msgs/Pose.h>

namespace monolithic_pr2_planner {
    class RobotPose {
        public:
            RobotPose(ContBaseState base_state, ContArmState r_arm, ContArmState l_arm);
            DiscBaseState getDiscBaseState() const { return m_base_state; };
            DiscBaseState getContBaseState() const;
            ContArmState getContRightArm() const { return m_right_arm; };
            ContArmState getContLeftArm() const { return m_left_arm; };

            unsigned int getLeftDiscFreeAngle() { return m_left_arm.getDiscFreeAngle(); };
            unsigned int getRightDiscFreeAngle() { return m_right_arm.getDiscFreeAngle(); };

        private:
            DiscBaseState m_base_state;
            ContArmState m_right_arm;
            ContArmState m_left_arm;
    };
}
