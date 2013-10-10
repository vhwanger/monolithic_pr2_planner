#include <vector>
#include <monolithic_pr2_planner/ParameterCatalog.h>
#include <monolithic_pr2_planner/MotionPrimitives/FileParser.h>
#include <monolithic_pr2_planner/MotionPrimitives/AdaptiveMotionPrimitives.h>
#include <monolithic_pr2_planner/MotionPrimitives/ArmMotionPrimitives.h>
#include <monolithic_pr2_planner/MotionPrimitives/BaseMotionPrimitives.h>

namespace monolithic_pr2_planner {
    class MotionPrimitivesMgr {
        public:
            //bool initialize(
        private:
            MotionPrimitiveFileParser m_parser;
            std::vector<ArmMotionPrimitives> m_arm_motprim;
            std::vector<BaseMotionPrimitives> m_base_motprim;
            std::vector<AdaptiveMotionPrimitives> m_adaptive_motprim;
    }
}
