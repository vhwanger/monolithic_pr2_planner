#pragma once
#include <vector>
#include <monolithic_pr2_planner/ParameterCatalog.h>
#include <monolithic_pr2_planner/MotionPrimitives/FileParser.h>
#include <monolithic_pr2_planner/MotionPrimitives/AdaptiveMotionPrimitive.h>
#include <monolithic_pr2_planner/MotionPrimitives/ArmMotionPrimitive.h>
#include <monolithic_pr2_planner/MotionPrimitives/BaseMotionPrimitive.h>

namespace monolithic_pr2_planner {
    class MotionPrimitivesMgr {
        public:
            bool loadMPrims(const MotionPrimitiveFiles& files);
        private:
            MotionPrimitiveFileParser m_parser;
            std::vector<MotionPrimitivePtr> m_motprim;
    };
}
