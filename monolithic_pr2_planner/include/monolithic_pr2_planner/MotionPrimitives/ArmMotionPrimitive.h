#pragma once
#include <monolithic_pr2_planner/MotionPrimitives/MotionPrimitive.h>
#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <vector>
namespace monolithic_pr2_planner {
    class ArmMotionPrimitive : public MotionPrimitive {
        public:
            void setGroup(int group) { m_group = group; };
            int getGroup() { return m_group; };
            virtual GraphStatePtr apply(GraphStatePtr graph_state);
            virtual void print();
        private:
            int m_group;
    };
    typedef boost::shared_ptr<ArmMotionPrimitive> ArmMotionPrimitivePtr;
}
