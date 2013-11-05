#pragma once
#include <monolithic_pr2_planner/MotionPrimitives/MotionPrimitive.h>
#include <boost/shared_ptr.hpp>

namespace monolithic_pr2_planner {
    class BaseMotionPrimitive : public MotionPrimitive { 
        public:
            void setStartAngle(int start_angle) { m_start_angle = start_angle; };

            void setCost(double cost) { m_additional_action_cost_mult = cost; };

            int getStartAngle(){ return m_start_angle; };

            int getCost(){ return m_additional_action_cost_mult; };
            virtual GraphStatePtr apply(GraphStatePtr graph_state);
            virtual void print();

        private:
            int m_start_angle;
            double m_additional_action_cost_mult;
    };
    typedef boost::shared_ptr<BaseMotionPrimitive> BaseMotionPrimitivePtr;
}
