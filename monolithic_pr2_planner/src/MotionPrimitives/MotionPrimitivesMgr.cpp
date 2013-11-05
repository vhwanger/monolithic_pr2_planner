#include <monolithic_pr2_planner/MotionPrimitives/MotionPrimitivesMgr.h>
#include <boost/foreach.hpp>

using namespace monolithic_pr2_planner;
using namespace std;

bool MotionPrimitivesMgr::loadMPrims(const MotionPrimitiveFiles& files){
    m_parser.parseArmMotionPrimitives(files.arm_motion_primitive_file, m_motprims);
    m_parser.parseBaseMotionPrimitives(files.base_motion_primitive_file, m_motprims);


    return true;
}

void MotionPrimitivesMgr::setCosts(){
    BOOST_FOREACH(auto mprim, m_motprims){
        // TODO: AHHHH fix this later
        mprim->setCost(1);
    }
}
