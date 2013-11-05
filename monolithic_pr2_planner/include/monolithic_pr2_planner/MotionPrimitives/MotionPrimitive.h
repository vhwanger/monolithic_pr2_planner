#pragma once
#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <monolithic_pr2_planner/Constants.h>
#include <assert.h>

namespace monolithic_pr2_planner {
    typedef std::vector<std::vector<double> > IntermSteps;
    class MotionPrimitive {
        public:
            MotionPrimitive();
            void setID(int id) { m_id = id; };
            int getID() { return m_id; };
            virtual void setIntermSteps(IntermSteps& coord) { m_interm_steps = coord; };
            virtual void setEndCoord(GraphStateMotion& coord); 
            virtual GraphStatePtr apply(GraphStatePtr graph_state) = 0;
            
            virtual void print() = 0;
            virtual void printIntermSteps();
            virtual void printEndCoord();

        protected:
            int m_id;
            GraphStateMotion m_end_coord;
            IntermSteps m_interm_steps;
    };
    typedef boost::shared_ptr<MotionPrimitive> MotionPrimitivePtr;
}
