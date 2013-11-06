#include <monolithic_pr2_planner/MotionPrimitives/ArmMotionPrimitive.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <boost/shared_ptr.hpp>

using namespace monolithic_pr2_planner;
using namespace boost;
using namespace std;

void ArmMotionPrimitive::print() const {
    ROS_DEBUG_NAMED(CONFIG_LOG, "Arm Primitive");
    ROS_DEBUG_NAMED(CONFIG_LOG, "\tgroup: %d", getGroup());
    ROS_DEBUG_NAMED(CONFIG_LOG, "\tid: %d", getID());
    ROS_DEBUG_NAMED(CONFIG_LOG, "\tcost: %d", getCost());
    printEndCoord();
    printIntermSteps();
}

bool ArmMotionPrimitive::apply(const GraphState& graph_state, 
                           unique_ptr<GraphState>& successor){
    successor.reset(new GraphState(graph_state));
    return successor->applyMPrim(m_end_coord);
}

void ArmMotionPrimitive::computeCost(const MotionPrimitiveParams& params){
    m_cost = 1;
}
