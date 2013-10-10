#pragma once
namespace monolithic_pr2_planner {
    class MotionPrimitiveFileParser {
        public:
            bool parseArmMotionPrimitives(FILE* mprim_file);
            bool parseBaseMotionPrimitives(FILE* mprim_file);
        private:
            char[] getNextToken(FILE* mprim_file);
    };
}
