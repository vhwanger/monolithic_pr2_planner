#pragma once
#include <monolithic_pr2_planner/StateReps/ContBaseState.h>
#include <monolithic_pr2_planner/Constants.h>
#include <vector>
namespace monolithic_pr2_planner {
    class ContBaseState;

    class DiscBaseState {
        public:
            DiscBaseState(unsigned int x, unsigned int y, 
                             unsigned int z, unsigned int theta);
            DiscBaseState(ContBaseState cont_body_pose);

            unsigned int getBodyX() const { return pose[X]; };
            unsigned int getBodyY() const { return pose[Y]; };
            unsigned int getBodyZ() const { return pose[Z]; };
            unsigned int getBodyTheta() const { return pose[THETA]; };

            void setBodyX(unsigned int x);
            void setBodyY(unsigned int y);
            void setBodyZ(unsigned int z);
            void setBodyTheta(unsigned int theta);

            ContBaseState getContBaseState();
        private:
            std::vector<unsigned int> pose;
    };
}
