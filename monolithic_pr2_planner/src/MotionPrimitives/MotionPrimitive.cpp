#include <monolithic_pr2_planner/MotionPrimitives/MotionPrimitive.h>
#include <boost/foreach.hpp>
#include <sstream>

using namespace monolithic_pr2_planner;
using namespace std;


void MotionPrimitive::printIntermSteps(){
    BOOST_FOREACH(auto step, m_interm_steps){
        stringstream ss;
        ss << "\t";
        BOOST_FOREACH(auto coord, step){
            ss << coord << " ";
        }
        ROS_DEBUG_NAMED(CONFIG_LOG, "\tinterm steps %s", ss.str().c_str());
    }
}

void MotionPrimitive::printEndCoord(){
    stringstream ss;
    ss << "\t";
    BOOST_FOREACH(auto coord, m_end_coord){
        ss << coord << " ";
    }
    ROS_DEBUG_NAMED(CONFIG_LOG, "\tend coord %s", ss.str().c_str());
}
