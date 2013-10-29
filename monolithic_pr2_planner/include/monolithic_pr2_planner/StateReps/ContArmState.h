#pragma once
#include <vector>
#include <monolithic_pr2_planner/Constants.h>
#include <monolithic_pr2_planner/ParameterCatalog.h>
#include <monolithic_pr2_planner/LoggerNames.h>
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


            unsigned int getDiscFreeAngle() const;
            double convertDiscFreeAngleToCont(unsigned int disc_angle) const;
            void getAngles(std::vector<double>* angles); 
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
            //static SBPLArmModelPtr m_arm_model;
            static KDL::Frame m_object_offset;
            static boost::shared_ptr<RobotResolutionParams> m_params;

            bool m_is_enforcing_joint_limits;
            std::vector<double> m_angles;
            ArmSide m_arm_side;
    };
    typedef boost::shared_ptr<ContArmState> ContArmStatePtr;

    // trying CRTP as described here:
    // http://stackoverflow.com/questions/12796580/static-variable-for-each-derived-class

    template<class Derived>
    class BaseX : public ContArmState {
        public:
            BaseX() : ContArmState() {}
            BaseX(std::vector<double> angles) : ContArmState(angles) {}


            static KDL::Frame getObjectOffset() { return m_object_offset; }
            static void setObjectOffset(KDL::Frame& object_offset){ m_object_offset = object_offset; };
    };

    class LeftContArmState : public BaseX<LeftContArmState> {
        public:
            LeftContArmState() : BaseX() { }
            static SBPLArmModelPtr getArmModel() { return m_arm_model; }
            LeftContArmState(std::vector<double> angles) : BaseX(angles) { }
            // TODO: figure out why i have to put this in header for it to compile
            static void setArmModel(ArmDescriptionParams& params){
                FILE* fp_arm = fopen(params.arm_file.c_str(), "r");
                if (!fp_arm){
                    ROS_ERROR_NAMED(CONFIG_LOG, "Couldn't open right arm model file (%s)!",
                               params.arm_file.c_str());
                }
                m_arm_model = boost::make_shared<sbpl_arm_planner::SBPLArmModel>(fp_arm);
                ROS_INFO_NAMED(CONFIG_LOG, "setting arm model resolution to %f", 
                               params.env_resolution);
                m_arm_model->setResolution(params.env_resolution);
                if (!params.robot_description_string.compare("ROS_PARAM")){
                    ROS_INFO_NAMED(CONFIG_LOG, "getting kdl chain from paramserver");
                    m_arm_model->initKDLChainFromParamServer();
                } else {
                    ROS_INFO_NAMED(CONFIG_LOG, "getting kdl chain from string");
                    m_arm_model->initKDLChain(params.robot_description_string);
                }
            }

        private:
            static SBPLArmModelPtr m_arm_model;
            static KDL::Frame m_object_offset;
    };

    class RightContArmState : public BaseX<RightContArmState> {
        public:
            RightContArmState() : BaseX() { }
            static SBPLArmModelPtr getArmModel() { return m_arm_model; }
            RightContArmState(std::vector<double> angles) : BaseX(angles) { }
            // TODO: figure out why i have to put this in header for it to compile
            static void setArmModel(ArmDescriptionParams& params){
                FILE* fp_arm = fopen(params.arm_file.c_str(), "r");
                if (!fp_arm){
                    ROS_ERROR_NAMED(CONFIG_LOG, "Couldn't open right arm model file (%s)!",
                               params.arm_file.c_str());
                }
                m_arm_model = boost::make_shared<sbpl_arm_planner::SBPLArmModel>(fp_arm);
                ROS_INFO_NAMED(CONFIG_LOG, "setting arm model resolution to %f", 
                               params.env_resolution);
                m_arm_model->setResolution(params.env_resolution);
                if (!params.robot_description_string.compare("ROS_PARAM")){
                    ROS_INFO_NAMED(CONFIG_LOG, "getting kdl chain from paramserver");
                    m_arm_model->initKDLChainFromParamServer();
                } else {
                    ROS_INFO_NAMED(CONFIG_LOG, "getting kdl chain from string");
                    m_arm_model->initKDLChain(params.robot_description_string);
                }
            }

        private:
            static SBPLArmModelPtr m_arm_model;
            static KDL::Frame m_object_offset;
    };
}
