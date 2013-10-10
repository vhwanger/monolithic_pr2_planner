#pragma once
#include <vector>
#include <monolithic_pr2_planner/Constants.h>
#include <monolithic_pr2_planner/IKManager.h>

namespace monolithic_pr2_planner {
    class ContArmState {
        public:
            ContArmState();

            void enableJointLimitChecking();

            inline double getShoulderPanAngle() const { return m_angles[SHOULDER_PAN]; };
            inline double getShoulderLiftAngle() const { return m_angles[SHOULDER_LIFT]; };
            inline double getUpperArmRollAngle() const { return m_angles[UPPER_ARM_ROLL]; };
            inline double getElbowFlexAngle() const { return m_angles[ELBOW_FLEX]; };
            inline double getForearmRollAngle() const { return m_angles[FOREARM_ROLL]; };
            inline double getWristFlexAngle() const { return m_angles[WRIST_FLEX]; };
            inline double getWristRollAngle() const { return m_angles[WRIST_ROLL]; };
            inline ArmSide getArm() const { return m_arm_side; };

            void setShoulderPan(double cont_value);
            void setShoulderLift(double cont_value);
            void setUpperArmRoll(double cont_value);
            void setElbowFlex(double cont_value);
            void setForearmRoll(double cont_value);
            void setWristFlex(double cont_value);
            void setWristRoll(double cont_value);
            void setArm(ArmSide arm);

            void getBodyFrameObjectPose();
            
        private:
            bool m_is_enforcing_joint_limits;
            IKManager m_ik_manager;
            std::vector<double> m_angles;
            ArmSide m_arm_side;
    };
}
