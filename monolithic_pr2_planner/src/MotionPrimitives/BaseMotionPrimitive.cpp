#include <monolithic_pr2_planner/MotionPrimitives/BaseMotionPrimitive.h>

using namespace monolithic_pr2_planner;
using namespace boost;
using namespace std;

void BaseMotionPrimitive::print() const {
    ROS_DEBUG_NAMED(CONFIG_LOG, "\tid: %d", getID());
    ROS_DEBUG_NAMED(CONFIG_LOG, "\tstart angle: %d", getStartAngle());
    ROS_DEBUG_NAMED(CONFIG_LOG, "\tcost: %d", getCost());
    printEndCoord();
    printIntermSteps();
}


unique_ptr<GraphState> BaseMotionPrimitive::apply(const GraphState& graph_state){
    unique_ptr<GraphState> successor (new GraphState(graph_state));
    successor->applyMPrim(m_end_coord);
    return successor;
}
