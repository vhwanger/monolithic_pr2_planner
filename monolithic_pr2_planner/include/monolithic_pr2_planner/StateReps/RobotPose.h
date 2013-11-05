#pragma once
#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <monolithic_pr2_planner/StateReps/ContBaseState.h>
#include <monolithic_pr2_planner/StateReps/DiscBaseState.h>
#include <monolithic_pr2_planner/StateReps/DiscObjectState.h>
#include <pviz/pviz.h>
#include <geometry_msgs/Pose.h>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>

namespace monolithic_pr2_planner {
    class RobotPose {
        public:
            bool operator==(const RobotPose& other);
            bool operator!=(const RobotPose& other);

            RobotPose(ContBaseState base_state, RightContArmState r_arm, LeftContArmState l_arm);
            DiscBaseState getDiscBaseState() const { return m_base_state; };
            ContBaseState getContBaseState();
            RightContArmState getContRightArm() const { return m_right_arm; };
            LeftContArmState getContLeftArm() const { return m_left_arm; };

            unsigned int getLeftDiscFreeAngle() const { return m_left_arm.getDiscFreeAngle(); };
            unsigned int getRightDiscFreeAngle() const { return m_right_arm.getDiscFreeAngle(); };
            void setLeftDiscFreeAngle(int value) { m_left_arm.setDiscFreeAngle(value); };
            void setRightDiscFreeAngle(int value) { m_right_arm.setDiscFreeAngle(value); };
            void setDiscBaseState(const DiscBaseState& base_state) { m_base_state = base_state; };

            void printToDebug(char* log_level) const;
            void printToInfo(char* log_level) const;

            static void setPViz(boost::shared_ptr<PViz> pviz);
            void visualize();

            static bool computeRobotPose(const DiscObjectState& disc_obj_state,
                                         const RobotPose& robot_pose,
                                         boost::shared_ptr<RobotPose>& new_robot_pose);
            ContObjectState getObjectStateRelMap() const;
            DiscObjectState getObjectStateRelBody() const;


        private:
            static boost::shared_ptr<PViz> m_pviz;
            DiscBaseState m_base_state;
            RightContArmState m_right_arm;
            LeftContArmState m_left_arm;
            DiscObjectState m_obj_state; // this is in BODY frame!
    };
    typedef boost::shared_ptr<RobotPose> RobotPosePtr;
}
