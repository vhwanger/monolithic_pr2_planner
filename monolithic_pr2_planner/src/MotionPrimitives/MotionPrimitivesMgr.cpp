#include <monolithic_pr2_planner/MotionPrimitives/MotionPrimitivesMgr.h>
#include <boost/foreach.hpp>

using namespace monolithic_pr2_planner;
using namespace std;

bool MotionPrimitivesMgr::loadMPrims(const MotionPrimitiveParams& params){
    m_params = params;
    m_parser.parseArmMotionPrimitives(params.arm_motion_primitive_file, m_motprims);
    m_parser.parseBaseMotionPrimitives(params.base_motion_primitive_file, m_motprims);
    setCosts();

    BOOST_FOREACH(auto mprim, m_motprims){    
        mprim->print();
    }

    return true;
}

void MotionPrimitivesMgr::setCosts(){
    BOOST_FOREACH(auto mprim, m_motprims){
        mprim->computeCost(m_params);
    }
}
