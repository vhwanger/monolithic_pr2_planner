#pragma once
#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <monolithic_pr2_planner/StateReps/ContBaseState.h>
#include <monolithic_pr2_planner/StateReps/DiscBaseState.h>
#include <monolithic_pr2_planner/StateReps/DiscObjectState.h>
#include <geometry_msgs/Pose.h>

namespace monolithic_pr2_planner {
    class RobotPose {
        public:
            bool operator==(const RobotPose& other);
            bool operator!=(const RobotPose& other);

            RobotPose(ContBaseState base_state, RightContArmState r_arm, LeftContArmState l_arm);
            DiscBaseState getDiscBaseState() const { return m_base_state; };
            ContBaseState getContBaseState();
            ContArmState getContRightArm() const { return m_right_arm; };
            ContArmState getContLeftArm() const { return m_left_arm; };

            unsigned int getLeftDiscFreeAngle() const { return m_left_arm.getDiscFreeAngle(); };
            unsigned int getRightDiscFreeAngle() const { return m_right_arm.getDiscFreeAngle(); };

            void printToDebug(char* log_level);
            void printToInfo(char* log_level);

            DiscObjectState getMapFrameObjectState();

        private:
            DiscBaseState m_base_state;
            RightContArmState m_right_arm;
            LeftContArmState m_left_arm;
    };
}
