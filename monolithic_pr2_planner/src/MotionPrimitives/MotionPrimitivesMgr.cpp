#include <monolithic_pr2_planner/MotionPrimitives/MotionPrimitivesMgr.h>
#include <boost/foreach.hpp>

using namespace monolithic_pr2_planner;
using namespace std;
using namespace boost;

MotionPrimitivesMgr::MotionPrimitivesMgr(boost::shared_ptr<GoalState>& goal){ }

bool MotionPrimitivesMgr::loadMPrims(const MotionPrimitiveParams& params){
    m_params = params;
    //m_parser.parseArmMotionPrimitives(params.arm_motion_primitive_file, m_motprims);
    m_parser.parseBaseMotionPrimitives(params.base_motion_primitive_file, m_motprims);

    //int NEG_TURN = -1;
    //int POS_TURN = 1;
    //BaseAdaptiveMotionPrimitivePtr bamp1 = make_shared<BaseAdaptiveMotionPrimitive>(NEG_TURN);
    //BaseAdaptiveMotionPrimitivePtr bamp2 = make_shared<BaseAdaptiveMotionPrimitive>(POS_TURN);
    //m_motprims.push_back(bamp1);
    //m_motprims.push_back(bamp2);

    computeAllMPrimCosts();

    BOOST_FOREACH(auto mprim, m_motprims){    
        mprim->print();
    }

    return true;
}

void MotionPrimitivesMgr::computeAllMPrimCosts(){
    BOOST_FOREACH(auto mprim, m_motprims){
        mprim->computeCost(m_params);
    }
}
