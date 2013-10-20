#include <monolithic_pr2_planner/StateReps/RobotPose.h>
#include <monolithic_pr2_planner/StateReps/DiscObjectState.h>
#include <monolithic_pr2_planner/StateReps/ContObjectState.h>

namespace monolithic_pr2_planner {
    class GraphState {
        public:
            unsigned int getStateID() const { return m_state_id; } ;
            RobotPose getRobotPose() const { return m_robot_state; } ;
            DiscObjectState getDiscObjectState() const { return m_disc_obj_pose;} ;
            ContObjectState getContObjectState();

        private:
            unsigned int m_state_id;
            RobotPose m_robot_state;
            DiscObjectState m_disc_obj_pose;
    };

};
