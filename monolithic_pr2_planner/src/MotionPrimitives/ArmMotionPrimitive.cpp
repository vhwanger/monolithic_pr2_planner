#include <monolithic_pr2_planner/MotionPrimitives/ArmMotionPrimitive.h>

using namespace monolithic_pr2_planner;

void ArmMotionPrimitive::print(){
    ROS_DEBUG_NAMED(CONFIG_LOG, "\tgroup: %d", getGroup());
    ROS_DEBUG_NAMED(CONFIG_LOG, "\tid: %d", getID());
    printEndCoord();
    printIntermSteps();
}


void ArmMotionPrimitive::apply(GraphStatePtr graph_state){
}
