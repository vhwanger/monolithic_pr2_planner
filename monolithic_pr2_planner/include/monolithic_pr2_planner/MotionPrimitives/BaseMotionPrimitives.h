#pragma once
#include <monolithic_pr2_planner/MotionPrimitives/MotionPrimitives.h>
namespace monolithic_pr2_planner {
    class BaseMotionPrimitives : public MotionPrimitives { 
        public:
            bool parseMotionPrimitives(FILE* mprim_file);
    };
}