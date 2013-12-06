#pragma once
#include <monolithic_pr2_planner/MotionPrimitives/MotionPrimitive.h>
#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <monolithic_pr2_planner/StateReps/ContBaseState.h>
#include <boost/shared_ptr.hpp>

namespace monolithic_pr2_planner {
    // moves the object up/down by moving the torso up/down
    class TorsoMotionPrimitive : public MotionPrimitive {
        public:
            TorsoMotionPrimitive(int vertical_direction);
            virtual bool apply(const GraphState& graph_state, 
                               GraphStatePtr& successor,
                               TransitionData& t_data);
            virtual void print() const;
            virtual int motion_type() const { return MPrim_Types::TORSO; }; 
            virtual void computeCost(const MotionPrimitiveParams& params);
    };
    typedef boost::shared_ptr<TorsoMotionPrimitive> TorsoMotionPrimitivePtr;
}
