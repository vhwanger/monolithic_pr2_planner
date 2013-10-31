#include <monolithic_pr2_planner/MotionPrimitives/MotionPrimitivesMgr.h>

using namespace monolithic_pr2_planner;

bool MotionPrimitivesMgr::loadMPrims(const MotionPrimitiveFiles& files){
    //FILE* base_mprim_file = fopen(files.base_motion_primitive_file, "r");
    //FILE* arm_mprim_file = fopen(files.arm_motion_primitive_file, "r");
    //m_parser.parseBaseMotionPrimitives(base_mprim_file);
    m_parser.parseArmMotionPrimitives(files.arm_motion_primitive_file, m_motprim);
    m_parser.parseBaseMotionPrimitives(files.base_motion_primitive_file, m_motprim);
    return true;
}
