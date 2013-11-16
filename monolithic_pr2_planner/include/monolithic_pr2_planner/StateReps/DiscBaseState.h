#pragma once
#include <monolithic_pr2_planner/StateReps/ContBaseState.h>
#include <monolithic_pr2_planner/Constants.h>
#include <monolithic_pr2_planner/OccupancyGridUser.h>
#include <pr2_collision_checker/pr2_collision_space.h>
#include <vector>
namespace monolithic_pr2_planner {
    class ContBaseState;

    class DiscBaseState : public OccupancyGridUser {
        public:
            DiscBaseState():m_state(4,0){};
            DiscBaseState(int x, int y, 
                    int z, int theta);
            DiscBaseState(ContBaseState cont_body_state);

            bool operator==(const DiscBaseState& other) const ;
            bool operator!=(const DiscBaseState& other) const ;

            int x() const { return m_state[BodyDOF::X]; };
            int y() const { return m_state[BodyDOF::Y]; };
            int z() const { return m_state[BodyDOF::Z]; };
            int theta() const { return m_state[BodyDOF::THETA]; };

            // TODO typo
            void geStateValues(std::vector<int>* values) const;
            std::vector<int>::const_iterator getCoordBegin(){ return m_state.begin(); };
            std::vector<int>::const_iterator getCoordEnd(){ return m_state.end(); };

            void x(int x){ m_state[BodyDOF::X] = x; };
            void y(int y){ m_state[BodyDOF::Y] = y; };
            void z(int z){ m_state[BodyDOF::Z] = z; };
            void theta(int theta){ m_state[BodyDOF::THETA] = normalizeTheta(theta); };

            static int convertContTheta(double theta);
            static int convertContDistance(double distance);

            ContBaseState getContBaseState() const;
            BodyPose getBodyPose() const;

        private:
            inline int normalizeTheta(int theta){
                int num_thetas = m_resolution_params.num_base_angles;
                if (theta >=0) 
                    return (theta % num_thetas);
                else
                    return (theta % num_thetas + num_thetas) % num_thetas;
            }
            std::vector<int> m_state;
    };
}
