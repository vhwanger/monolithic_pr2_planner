#pragma once
#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <monolithic_pr2_planner/StateReps/RobotState.h>
#include <monolithic_pr2_planner/StateReps/ContBaseState.h>
#include <vector>

namespace monolithic_pr2_planner {
    typedef std::vector<std::vector<double> > IntermSteps;
    /*! \brief Contains information generated during a state expansion.
     *
     * When a state is expanded, it usually has unique information about the
     * expansion, namely the intermediate states associated with the particular
     * motion primitive. This class wraps it all up so it can be easily used for
     * collision checking and path reconstruction by other objects.
     */
    class TransitionData {
        public:
            TransitionData(){};
            void successor_id(int id){ m_successor_id = id; };
            int successor_id(){ return m_successor_id; };
            void motion_type(int mt){ m_motion_type = mt; };
            int motion_type() const { return m_motion_type; };

            void interm_robot_steps(std::vector<RobotState> steps){ m_robot_interm_steps = steps; };
            std::vector<RobotState> interm_robot_steps() const { return m_robot_interm_steps; };

            void cont_base_interm_steps(std::vector<ContBaseState> steps){ m_cont_base_interm_steps = steps; };
            std::vector<ContBaseState> cont_base_interm_steps() const { return m_cont_base_interm_steps; };

            void cost(int cost){ m_cost = cost; };
            int cost() const { return m_cost; };
        private:
            int m_successor_id;
            int m_cost;

            // I'm providing two kinds of intermediate steps because I'm not
            // sure which one will be more useful...
            IntermSteps m_graph_interm_steps;
            std::vector<RobotState> m_robot_interm_steps;
            std::vector<ContBaseState> m_cont_base_interm_steps;
            int m_motion_type;
    };
}
