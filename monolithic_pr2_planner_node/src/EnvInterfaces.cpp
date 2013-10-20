#include <monolithic_pr2_planner_node/EnvInterfaces.h>
#include <boost/filesystem.hpp>
#include <geometry_msgs/Pose.h>
#include <leatherman/utils.h>
#include <LinearMath/btVector3.h>

using namespace monolithic_pr2_planner_node;
using namespace monolithic_pr2_planner;
using namespace boost::filesystem;
using namespace std;

EnvInterfaces::EnvInterfaces(Environment& env) : 
    m_nodehandle("~"), m_env(boost::make_shared<Environment>(env)),
    m_collision_space_interface(env.getCollisionSpace()){
        getParams();
}

void EnvInterfaces::getParams(){
    m_nodehandle.param<std::string>("reference_frame", m_params.ref_frame, std::string("map"));
}

bool EnvInterfaces::bindPlanPathService(){
    return true;
}

bool EnvInterfaces::bindCollisionSpaceToTopic(std::string topic_name){
    m_collision_space_interface.bindCollisionSpaceToTopic(topic_name, m_tf, m_params.ref_frame);
    return true;
}

