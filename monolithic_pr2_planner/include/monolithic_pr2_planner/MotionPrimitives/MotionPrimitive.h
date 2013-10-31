#pragma once
#include <monolithic_pr2_planner/StateReps/GraphState.h>

namespace monolithic_pr2_planner {
    typedef std::vector<std::vector<double> > IntermSteps;
    class MotionPrimitive {
        public:
            void setID(int id) { m_id = id; };
            int getID() { return m_id; };
            virtual void setIntermSteps(IntermSteps& coord) { m_interm_steps = coord; };
            virtual void setEndCoord(std::vector<int>& coord) { m_end_coord = coord; };
            virtual void apply(GraphStatePtr graph_state) = 0;
            
            virtual void print() = 0;
            virtual void printIntermSteps();
            virtual void printEndCoord();

        private:
            int m_id;
            std::vector<int> m_end_coord;
            IntermSteps m_interm_steps;
    };
    typedef boost::shared_ptr<MotionPrimitive> MotionPrimitivePtr;
}
