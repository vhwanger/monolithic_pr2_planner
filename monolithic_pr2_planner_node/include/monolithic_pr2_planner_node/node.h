#include <ros/ros.h>
#include <monolithic_pr2_planner/Environment.h>
#include <monolithic_pr2_planner_node/EnvInterfaces.h>
#include <boost/shared_ptr.hpp>

namespace monolithic_pr2_planner_node {
    class Node {
        public:
            Node();
        private:
            monolithic_pr2_planner::Environment m_env;
            EnvInterfaces m_env_interface;
    };
} 
