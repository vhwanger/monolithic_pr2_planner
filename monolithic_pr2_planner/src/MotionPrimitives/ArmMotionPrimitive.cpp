#include <monolithic_pr2_planner/MotionPrimitives/ArmMotionPrimitive.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <boost/shared_ptr.hpp>

using namespace monolithic_pr2_planner;
using namespace boost;
using namespace std;

void ArmMotionPrimitive::print() const {
    ROS_DEBUG_NAMED(CONFIG_LOG, "\tgroup: %d", getGroup());
    ROS_DEBUG_NAMED(CONFIG_LOG, "\tid: %d", getID());
    printEndCoord();
    printIntermSteps();
}


unique_ptr<GraphState> ArmMotionPrimitive::apply(const GraphState& graph_state){
    unique_ptr<GraphState> successor (new GraphState(graph_state));
    successor->applyMPrim(m_end_coord);
    return successor;
}
