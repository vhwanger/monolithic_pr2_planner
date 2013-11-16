#pragma once
#include <monolithic_pr2_planner/MotionPrimitives/MotionPrimitive.h>
#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <monolithic_pr2_planner/StateReps/GoalState.h>
#include <monolithic_pr2_planner/Constants.h>
#include <boost/shared_ptr.hpp>

namespace monolithic_pr2_planner {
    class ArmAdaptiveMotionPrimitive : public MotionPrimitive {
        public:
            virtual bool apply(const GraphState& graph_state, 
                               GraphStatePtr& successor);
            virtual void print() const;
            virtual int motion_type() const { return MPrim_Types::ARM; }; 
            virtual void computeCost(const MotionPrimitiveParams& params);

            // TODO yuck
            static void goal(GoalState& goal) { m_goal = goal; };
            static GoalState goal() { return m_goal; };
            
        private:
            // TODO yuck?
            static GoalState m_goal;
    };
    typedef boost::shared_ptr<ArmAdaptiveMotionPrimitive> ArmAdaptiveMotionPrimitivePtr;

}
