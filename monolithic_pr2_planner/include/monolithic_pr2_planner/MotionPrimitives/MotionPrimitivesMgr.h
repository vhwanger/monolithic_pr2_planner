#pragma once
#include <vector>
#include <monolithic_pr2_planner/ParameterCatalog.h>
#include <monolithic_pr2_planner/MotionPrimitives/FileParser.h>
#include <monolithic_pr2_planner/MotionPrimitives/BaseAdaptiveMotionPrimitive.h>
#include <monolithic_pr2_planner/MotionPrimitives/ArmAdaptiveMotionPrimitive.h>
#include <monolithic_pr2_planner/MotionPrimitives/ArmMotionPrimitive.h>
#include <monolithic_pr2_planner/MotionPrimitives/BaseMotionPrimitive.h>
#include <monolithic_pr2_planner/MotionPrimitives/TorsoMotionPrimitive.h>
#include <monolithic_pr2_planner/StateReps/GoalState.h>
#include <monolithic_pr2_planner/StateReps/GraphState.h>

namespace monolithic_pr2_planner {
    class MotionPrimitivesMgr {
        public:
            MotionPrimitivesMgr(){};
            MotionPrimitivesMgr(GoalStatePtr& goal);
            bool loadMPrims(const MotionPrimitiveParams& files);
            std::vector<MotionPrimitivePtr> getMotionPrims() { return m_motprims; };
        private:
            void computeAllMPrimCosts();
            double dist(DiscObjectState s1, DiscObjectState s2);
            MotionPrimitiveFileParser m_parser;
            std::vector<MotionPrimitivePtr> m_motprims;
            MotionPrimitiveParams m_params;
            GoalStatePtr m_goal;
    };
}
