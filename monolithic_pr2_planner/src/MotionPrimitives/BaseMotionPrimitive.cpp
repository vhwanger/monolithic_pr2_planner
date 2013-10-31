#include <monolithic_pr2_planner/MotionPrimitives/BaseMotionPrimitive.h>

using namespace monolithic_pr2_planner;

void BaseMotionPrimitive::print(){
    ROS_DEBUG_NAMED(CONFIG_LOG, "\tid: %d", getID());
    ROS_DEBUG_NAMED(CONFIG_LOG, "\tstart angle: %d", getStartAngle());
    ROS_DEBUG_NAMED(CONFIG_LOG, "\tcost: %d", getCost());
    printEndCoord();
    printIntermSteps();
}


void BaseMotionPrimitive::apply(GraphStatePtr graph_state){
}
