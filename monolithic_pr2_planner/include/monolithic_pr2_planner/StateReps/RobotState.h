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
    class RobotState {
        public:
            bool operator==(const RobotState& other);
            bool operator!=(const RobotState& other);

            RobotState(){};
            RobotState(ContBaseState base_state, RightContArmState r_arm, LeftContArmState l_arm);
            DiscBaseState base_state() const { return m_base_state; };
            ContBaseState getContBaseState();
            RightContArmState right_arm() const { return m_right_arm; };
            LeftContArmState left_arm() const { return m_left_arm; };

            unsigned int left_free_angle() const { return m_left_arm.getDiscFreeAngle(); };
            unsigned int right_free_angle() const { return m_right_arm.getDiscFreeAngle(); };
            void left_free_angle(int value) { m_left_arm.setDiscFreeAngle(value); };
            void right_free_angle(int value) { m_right_arm.setDiscFreeAngle(value); };
            void base_state(const DiscBaseState& base_state) { m_base_state = base_state; };

            void printToDebug(char* log_level) const;
            void printToInfo(char* log_level) const;

            //static void setPViz(boost::shared_ptr<PViz> pviz);
            void visualize();

            ContObjectState getObjectStateRelMap() const;
            DiscObjectState getObjectStateRelBody() const;

            static bool computeRobotPose(const DiscObjectState& disc_obj_state,
                                         const RobotState& robot_pose,
                                         boost::shared_ptr<RobotState>& new_robot_pose);
            //static bool interpolateRobotPose(const RobotPose& start, const RobotPose& end,
            //                                 int num_steps, std::vector<RobotPose>* interp_steps);




        private:
            //static boost::shared_ptr<PViz> m_pviz;
            DiscBaseState m_base_state;
            RightContArmState m_right_arm;
            LeftContArmState m_left_arm;
            DiscObjectState m_obj_state; // this is in BODY frame!
    };
    typedef boost::shared_ptr<RobotState> RobotPosePtr;
}
