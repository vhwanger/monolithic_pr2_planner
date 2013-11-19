#pragma once
#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <monolithic_pr2_planner/StateReps/RobotState.h>
#include <vector>

namespace monolithic_pr2_planner {
    typedef std::vector<std::vector<double> > IntermSteps;
    class TransitionData {
        public:
            TransitionData(){};
            void successor_id(int id){ m_successor_id = id; };
            void motion_type(int mt){ m_motion_type = mt; };
            void interm_robot_steps(std::vector<RobotState> steps){ m_robot_interm_steps = steps; };
        private:
            int m_successor_id;

            // I'm providing two kinds of intermediate steps because I'm not
            // sure which one will be more useful...
            IntermSteps m_graph_interm_steps;
            std::vector<RobotState> m_robot_interm_steps;
            int m_motion_type;
    };
}
