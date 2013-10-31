#pragma once
#include <monolithic_pr2_planner/MotionPrimitives/ArmMotionPrimitive.h>
#include <monolithic_pr2_planner/MotionPrimitives/BaseMotionPrimitive.h>
#include <vector>
namespace monolithic_pr2_planner {
    class MotionPrimitiveFileParser {
        public:
            bool parseArmMotionPrimitives(std::string filename,
                                          std::vector<MotionPrimitivePtr>& prims);
            bool parseBaseMotionPrimitives(std::string filename,
                                          std::vector<MotionPrimitivePtr>& prims);
        private:
            void getNextLine(ifstream& file, stringstream& ss, string& line);
            bool ReadinMotionPrimitive(BaseMotionPrimitive* pMotPrim, FILE* fIn,
                                       int num_base_dirs);
    };
}
