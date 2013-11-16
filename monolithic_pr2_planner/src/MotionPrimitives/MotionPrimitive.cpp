#include <monolithic_pr2_planner/MotionPrimitives/MotionPrimitive.h>
#include <boost/foreach.hpp>
#include <sstream>

using namespace monolithic_pr2_planner;
using namespace std;

MotionPrimitive::MotionPrimitive() : m_end_coord(GRAPH_STATE_SIZE,0){
}

void MotionPrimitive::setEndCoord(GraphStateMotion& coord) { 
    assert((int)coord.size()==GRAPH_STATE_SIZE); 
    m_end_coord = coord; 
}

void MotionPrimitive::printIntermSteps() const {
    BOOST_FOREACH(auto step, m_interm_steps){
        stringstream ss;
        ss << "\t";
        BOOST_FOREACH(auto coord, step){
            ss << coord << " ";
        }
        ROS_DEBUG_NAMED(MPRIM_LOG, "\tinterm steps %s", ss.str().c_str());
    }
}

void MotionPrimitive::printEndCoord() const {
    stringstream ss;
    ss << "\t";
    BOOST_FOREACH(auto coord, m_end_coord){
        ss << coord << " ";
    }
    ROS_DEBUG_NAMED(MPRIM_LOG, "\tend coord %s", ss.str().c_str());
}



double MotionPrimitive::dist(DiscObjectState s1, DiscObjectState s2){
    return pow(pow(s1.x()-s2.x(),2)+pow(s1.y()-s2.y(),2)+pow(s1.z()-s2.z(),2),.5);
}
