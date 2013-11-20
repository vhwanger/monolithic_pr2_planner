#pragma once
#include <monolithic_pr2_planner/MotionPrimitives/MotionPrimitive.h>
#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <monolithic_pr2_planner/Constants.h>
#include <vector>
#include <memory>

namespace monolithic_pr2_planner {
    class ArmMotionPrimitive : public MotionPrimitive {
        public:
            void setGroup(int group) { m_group = group; };
            int getGroup() const { return m_group; };
            virtual bool apply(const GraphState& graph_state, 
                               GraphStatePtr& successor,
                               TransitionData& t_data);
            virtual void print() const ;
            virtual int motion_type() const { return MPrim_Types::ARM; };
            virtual void computeCost(const MotionPrimitiveParams& params);

            void computeIntermSteps(const GraphState& source_state, 
                                    const GraphState& successor, 
                                    TransitionData& t_data);
        private:
            int m_group;
    };
    typedef boost::shared_ptr<ArmMotionPrimitive> ArmMotionPrimitivePtr;
}
