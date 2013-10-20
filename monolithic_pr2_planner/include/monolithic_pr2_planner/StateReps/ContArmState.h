#pragma once
#include <vector>
#include <monolithic_pr2_planner/Constants.h>
#include <monolithic_pr2_planner/ArmModel.h>
#include <monolithic_pr2_planner/ParameterCatalog.h>
#include <boost/shared_ptr.hpp>

namespace monolithic_pr2_planner {
    class ArmModel;
    class ContArmState {
        public:
            ContArmState();
            static void setRobotResolutionParams(const RobotResolutionParams& params);

            void enableJointLimitChecking();

            inline double getShoulderPanAngle() const{return m_angles[Joints::SHOULDER_PAN]; };
            inline double getShoulderLiftAngle() const{return m_angles[Joints::SHOULDER_LIFT];};
            inline double getUpperArmRollAngle() const {return m_angles[Joints::UPPER_ARM_ROLL];};
            inline double getElbowFlexAngle() const { return m_angles[Joints::ELBOW_FLEX]; };
            inline double getForearmRollAngle() const { return m_angles[Joints::FOREARM_ROLL]; };
            inline double getWristFlexAngle() const { return m_angles[Joints::WRIST_FLEX]; };
            inline double getWristRollAngle() const { return m_angles[Joints::WRIST_ROLL]; };


            unsigned int getDiscFreeAngle();
            double convertDiscFreeAngleToCont(unsigned int disc_angle);
            void getVectorOfAngles(std::vector<double>* angles); 
            std::vector<double>::const_iterator getAnglesBegin() const{return m_angles.begin();};
            std::vector<double>::const_iterator getAnglesEnd() const { return m_angles.end(); };

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
            boost::shared_ptr<ArmModel> m_arm_model;
            static boost::shared_ptr<RobotResolutionParams> m_params;

            bool m_is_enforcing_joint_limits;
            std::vector<double> m_angles;
            ArmSide m_arm_side;
    };
}
