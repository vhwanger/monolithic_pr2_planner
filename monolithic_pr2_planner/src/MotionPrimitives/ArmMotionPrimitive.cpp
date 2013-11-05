#include <monolithic_pr2_planner/MotionPrimitives/ArmMotionPrimitive.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <boost/shared_ptr.hpp>

using namespace monolithic_pr2_planner;
using namespace boost;

void ArmMotionPrimitive::print(){
    ROS_DEBUG_NAMED(CONFIG_LOG, "\tgroup: %d", getGroup());
    ROS_DEBUG_NAMED(CONFIG_LOG, "\tid: %d", getID());
    printEndCoord();
    printIntermSteps();
}


GraphStatePtr ArmMotionPrimitive::apply(GraphStatePtr graph_state){
    GraphStatePtr successor = make_shared<GraphState>(*graph_state);
    successor->applyMPrim(m_end_coord);
    ROS_DEBUG_NAMED(SEARCH_LOG, "successor state:");
    successor->printToDebug(SEARCH_LOG);
    return successor;
}
