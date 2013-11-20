#pragma once
#include <monolithic_pr2_planner/MotionPrimitives/MotionPrimitive.h>
#include <monolithic_pr2_planner/Constants.h>
#include <boost/shared_ptr.hpp>

namespace monolithic_pr2_planner {
    // The base motion primitives read in from file are different than normal
    // mprims. Instead of having 8 generic motion primitives that are applied at
    // each angle, we have 16*8 total mprims - 8 motion primitives for each 16
    // thetas. This is done because applying a generic motion to any theta may
    // not result in the same absolute movement due to discretization error.
    //
    class BaseMotionPrimitive : public MotionPrimitive { 
        public:
            void start_angle(int start_angle) { m_start_angle = start_angle; };
            int start_angle() const { return m_start_angle; };

            virtual bool apply(const GraphState& graph_state,
                               GraphStatePtr& successor,
                               TransitionData& t_data);
            virtual void print() const ;

            virtual int motion_type() const { return MPrim_Types::BASE; };
            virtual void computeCost(const MotionPrimitiveParams& params);

        private:
            int m_start_angle;
    };
    typedef boost::shared_ptr<BaseMotionPrimitive> BaseMotionPrimitivePtr;
}
