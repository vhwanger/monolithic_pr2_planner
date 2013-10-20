#include <monolithic_pr2_planner_node/node.h>
#include <ros/console.h>

using namespace monolithic_pr2_planner_node;
using namespace monolithic_pr2_planner;

Node::Node() : m_env_interface(m_env){
    m_env_interface.bindCollisionSpaceToTopic("collision_map_out");
}

int main(int argc, char** argv){
    ros::init(argc, argv, "dual_arm_node");
    Node node;

    ros::spin();
}
