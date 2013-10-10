#pragma once
#include <monolithic_pr2_planner/StateReps/DiscBaseState.h>
#include <monolithic_pr2_planner/Constants.h>
#include <vector>
namespace monolithic_pr2_planner {
    class DiscBaseState;
    class ContBaseState {
        public:
            ContBaseState(double x, double y, double z, double theta);
            ContBaseState(DiscBaseState disc_body_pose);

            double getBodyX() const { return pose[X]; };
            double getBodyY() const { return pose[Y]; };
            double getBodyZ() const { return pose[Z]; };
            double getBodyTheta() const { return pose[THETA]; };

            void setBodyX(double x);
            void setBodyY(double y);
            void setBodyZ(double z);
            void setBodyTheta(double theta);

            DiscBaseState getDiscBaseState();
        private:
            std::vector<double> pose;
    };
}
