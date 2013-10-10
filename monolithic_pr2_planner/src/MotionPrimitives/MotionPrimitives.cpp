#pragma once
#include <vector>
#include <monolithic_pr2_planner/StateReps/GraphState.h>

namespace monolithic_pr2_planner {
    class MotionPrimitiveFileParser {
        public:
            bool verifyFileFormat(const std::string filename);
            bool parseMotionPrimitives(FILE* mprim_file);
        private:
            char[] getNextToken(FILE* mprim_file);
    };

    class MotionPrimitives {
        public:
            bool apply(GraphState graph_state) = 0;
    };
}
