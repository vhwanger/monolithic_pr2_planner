#pragma once
#include <monolithic_pr2_planner/StateReps/RobotState.h>
#include <monolithic_pr2_planner/StateReps/DiscObjectState.h>
#include <monolithic_pr2_planner/StateReps/ContObjectState.h>

namespace monolithic_pr2_planner {
    typedef std::vector<int> GraphStateMotion;
    class GraphState {
        public:
            GraphState(RobotState robot_pose);
            GraphState(DiscObjectState obj_state, RobotState robot_pose);
            // equality of graphstates is defined as:
            //      same discrete base state
            //      same discrete object state
            //      same left and right arm discrete free angles
            bool operator==(const GraphState& other);
            bool operator!=(const GraphState& other);
            int id() const { return m_id; };
            void id(int id) { m_id = id; };
            RobotState robot_pose() const { return m_robot_pose; };
            void robot_pose(RobotState robot_state) { m_robot_pose = robot_state; };

            bool applyMPrim(const GraphStateMotion& mprim);

            void printToDebug(char* logger) const ;

            DiscObjectState getObjectStateRelMap() const;
            DiscObjectState getObjectStateRelBody() const;

        private:
            int m_id;
            RobotState m_robot_pose;
    };
    typedef boost::shared_ptr<GraphState> GraphStatePtr;

};
