#pragma once
#include <monolithic_pr2_planner/MotionPrimitives/MotionPrimitive.h>
#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <vector>
namespace monolithic_pr2_planner {
    class ArmMotionPrimitive : public MotionPrimitive {
        public:
            void setGroup(int group) { m_group = group; };
            int getGroup() const { return m_group; };
            virtual std::unique_ptr<GraphState> apply(const GraphState& graph_state);
            virtual void print() const ;
        private:
            int m_group;
    };
    typedef boost::shared_ptr<ArmMotionPrimitive> ArmMotionPrimitivePtr;
}
