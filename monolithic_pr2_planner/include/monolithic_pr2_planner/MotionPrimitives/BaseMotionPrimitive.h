#pragma once
#include <monolithic_pr2_planner/MotionPrimitives/MotionPrimitive.h>
#include <monolithic_pr2_planner/Constants.h>
#include <boost/shared_ptr.hpp>

namespace monolithic_pr2_planner {
    class BaseMotionPrimitive : public MotionPrimitive { 
        public:
            void setStartAngle(int start_angle) { m_start_angle = start_angle; };

            void setCost(double cost) { m_additional_action_cost_mult = cost; };

            int getStartAngle() const { return m_start_angle; };

            int getCost() const { return m_additional_action_cost_mult; };
            virtual std::unique_ptr<GraphState> apply(const GraphState& graph_state);
            virtual void print() const ;

            virtual int getMotionType() const { return MPrim_Types::BASE; };

        private:
            int m_start_angle;
            double m_additional_action_cost_mult;
    };
    typedef boost::shared_ptr<BaseMotionPrimitive> BaseMotionPrimitivePtr;
}
