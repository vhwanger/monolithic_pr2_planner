#pragma once
#include <vector>
#include <monolithic_pr2_planner/ParameterCatalog.h>
#include <monolithic_pr2_planner/MotionPrimitives/FileParser.h>
#include <monolithic_pr2_planner/MotionPrimitives/BaseAdaptiveMotionPrimitive.h>
#include <monolithic_pr2_planner/MotionPrimitives/ArmMotionPrimitive.h>
#include <monolithic_pr2_planner/MotionPrimitives/BaseMotionPrimitive.h>
#include <monolithic_pr2_planner/StateReps/GoalState.h>

namespace monolithic_pr2_planner {
    class MotionPrimitivesMgr {
        public:
            MotionPrimitivesMgr(){};
            MotionPrimitivesMgr(boost::shared_ptr<std::vector<GoalState> >& goals);
            bool loadMPrims(const MotionPrimitiveParams& files);
            std::vector<MotionPrimitivePtr> getMotionPrims() { return m_motprims; };
        private:
            void computeAllMPrimCosts();
            MotionPrimitiveFileParser m_parser;
            std::vector<MotionPrimitivePtr> m_motprims;
            MotionPrimitiveParams m_params;
            boost::weak_ptr<std::vector<GoalState> > m_goals;
    };
}
