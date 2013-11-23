#pragma once
#include <monolithic_pr2_planner/MotionPrimitives/MotionPrimitive.h>
#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <monolithic_pr2_planner/StateReps/GoalState.h>
#include <monolithic_pr2_planner/Constants.h>
#include <boost/shared_ptr.hpp>

namespace monolithic_pr2_planner {
    /*! \brief An adaptive motion that snaps the gripper to the object pose.
     *
     * The arm adaptive motion dynamically generates a motion that moves the current
     * source_state directly to the goal state. This only runs if the Euclidean
     * distance of the object state is sufficiently close to the goal - then all
     * that's left to do is align the roll, pitch, and yaw to the goal state. This
     * can fail in the yaw of the object is drastically different from the
     * source_state's yaw (in that case, the base probably needs to be moved, which
     * is done by the BaseAdaptiveMotionPrimitive). 
     */
    class ArmAdaptiveMotionPrimitive : public MotionPrimitive {
        public:
            virtual bool apply(const GraphState& graph_state, 
                               GraphStatePtr& successor,
                               TransitionData& t_data);
            virtual void print() const;
            virtual int motion_type() const { return MPrim_Types::ARM_ADAPTIVE; }; 
            virtual void computeCost(const MotionPrimitiveParams& params);

            // TODO yuck
            static void goal(GoalState& goal) { m_goal = goal; };
            static GoalState goal() { return m_goal; };
            
        private:
            // TODO yuck?
            void computeIntermSteps(const GraphState& source_state,
                                    const GraphState& successor,
                                    TransitionData& t_data);
            static GoalState m_goal;
    };
    typedef boost::shared_ptr<ArmAdaptiveMotionPrimitive> ArmAdaptiveMotionPrimitivePtr;

}
