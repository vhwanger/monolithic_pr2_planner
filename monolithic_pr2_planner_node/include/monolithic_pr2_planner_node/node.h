#include <ros/ros.h>
#include <monolithic_pr2_planner/Environment.h>
#include <boost/shared_ptr.hpp>

namespace monolithic_pr2_planner_node {
    class Node {
        public:
            bool init();
        private:
            monolithic_pr2_planner::Environment m_env;
    };
} 
