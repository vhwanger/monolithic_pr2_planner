#pragma once
#include <vector>
#include <monolithic_pr2_planner/Constants.h>
#include <monolithic_pr2_planner/ParameterCatalog.h>
#include <pr2_collision_checker/sbpl_arm_model.h>
#include <boost/shared_ptr.hpp>

namespace monolithic_pr2_planner {
    typedef boost::shared_ptr<sbpl_arm_planner::SBPLArmModel> SBPLArmModelPtr;
    class ContArmState {
        public:
            ContArmState();
            ContArmState(std::vector<double> arm_state);

            bool operator==(const ContArmState& other);
            bool operator!=(const ContArmState& other);

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

            void getBodyFrameObjectState();
            

        private:

            static boost::shared_ptr<RobotResolutionParams> m_params;
            bool m_is_enforcing_joint_limits;
            std::vector<double> m_angles;
            ArmSide m_arm_side;
    };
    typedef boost::shared_ptr<ContArmState> ContArmStatePtr;

    class LeftContArmState : public ContArmState {
        public:
            LeftContArmState(std::vector<double> angles) : ContArmState(angles) { }
            virtual SBPLArmModelPtr getArmModel(){ return m_arm_model; };
            static void setArmModel(const HardwareDescriptionFiles& params);

        private:
            static SBPLArmModelPtr m_arm_model;
    };

    class RightContArmState : public ContArmState {
        public:
            RightContArmState(std::vector<double> angles) : ContArmState(angles) { }
            virtual SBPLArmModelPtr getArmModel(){ return m_arm_model; };
            static void setArmModel(const HardwareDescriptionFiles& params);

        private:
            static SBPLArmModelPtr m_arm_model;
    };

    class ArmStateFactory {
        public:
            static boost::shared_ptr<ContArmState> createArmState(int arm_side,
                                                                  std::vector<double> angles);
    };
}