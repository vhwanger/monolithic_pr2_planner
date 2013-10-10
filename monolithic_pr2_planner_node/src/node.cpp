#include <monolithic_pr2_planner_node/node.h>
#include <ros/console.h>

using namespace monolithic_pr2_planner_node;
using namespace monolithic_pr2_planner;

bool Node::init(){
    m_env.init();
    return true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "dual_arm_node");
    Node node;

    ros::spin();
}
