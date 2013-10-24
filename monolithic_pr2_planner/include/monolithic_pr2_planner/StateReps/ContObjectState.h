#pragma once
#include <monolithic_pr2_planner/StateReps/DiscObjectState.h>
#include <monolithic_pr2_planner/OccupancyGridUser.h>
#include <geometry_msgs/PoseStamped.h>

namespace monolithic_pr2_planner {
    class DiscObjectState;
    class ContObjectState {
        public:
            ContObjectState(); 
            ContObjectState(DiscObjectState obj_state); 
            ContObjectState(geometry_msgs::PoseStamped obj_pose); 
            std::vector<double>::const_iterator getCoordBegin(){ return m_coord.begin(); };
            std::vector<double>::const_iterator getCoordEnd(){ return m_coord.end(); };
        private:
            std::vector<double> m_coord;
    };
}
