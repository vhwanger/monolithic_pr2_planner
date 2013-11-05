#pragma once
#include <monolithic_pr2_planner/StateReps/RobotPose.h>
#include <monolithic_pr2_planner/StateReps/DiscObjectState.h>
#include <monolithic_pr2_planner/StateReps/ContObjectState.h>

namespace monolithic_pr2_planner {
    typedef std::vector<int> GraphStateMotion;
    class GraphState {
        public:
            GraphState(RobotPose robot_pose);
            GraphState(DiscObjectState obj_state, RobotPose robot_pose);
            // equality of graphstates is defined as:
            //      same discrete base state
            //      same discrete object state
            //      same left and right arm discrete free angles
            bool operator==(const GraphState& other);
            bool operator!=(const GraphState& other);
            unsigned int getID() const { return m_id; };
            void setID(unsigned int id) { m_id = id; };
            RobotPose getRobotPose() const { return m_robot_pose; };

            bool applyMPrim(const GraphStateMotion& mprim);

            void printToDebug(char* logger);

            DiscObjectState getObjectStateRelMap();
            DiscObjectState getObjectStateRelBody();

        private:
            unsigned int m_id;
            RobotPose m_robot_pose;
    };
    typedef boost::shared_ptr<GraphState> GraphStatePtr;

};
