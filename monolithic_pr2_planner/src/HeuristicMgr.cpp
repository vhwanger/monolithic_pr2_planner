#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <monolithic_pr2_planner/OccupancyGridUser.h>
#include <monolithic_pr2_planner/StateReps/GoalState.h>
#include <monolithic_pr2_planner/HeuristicMgr.h>
#include <monolithic_pr2_planner/BFS3DHeuristic.h>
#include <memory>
#include <vector>
#include <boost/shared_ptr.hpp>

using namespace monolithic_pr2_planner;
using namespace boost;

int HeuristicMgr::add3DHeur(){
    AbstractHeuristicPtr new_3d_heur = make_shared<BFS3DHeuristic>();
    // Add it to the list of heuristics
    m_heuristics.push_back(new_3d_heur);
    return m_heuristics.size() - 1;
}

int HeuristicMgr::add2DHeur(){
    // Not implemented yet.
    return 0;
}

void HeuristicMgr::update2DHeuristicMaps(const std::vector<signed char>& data){
    for (size_t i = 0; i < m_heuristics.size(); ++i){
        m_heuristics[i]->update2DHeuristicMap(data);
    }
}

void HeuristicMgr::update3DHeuristicMaps(){
    for (size_t i = 0; i < m_heuristics.size(); ++i){
        m_heuristics[i]->update3DHeuristicMap();
    }
}

void HeuristicMgr::setGoal(GoalState& state){
    for (size_t i = 0; i < m_heuristics.size(); ++i){
        m_heuristics[i]->setGoal(state);
    }
}

std::vector<int> HeuristicMgr::getGoalHeuristic(const GraphStatePtr& state){
    if (!m_heuristics.size()){
        ROS_ERROR_NAMED(HEUR_LOG, "No heuristics initialized!");
    }
    std::vector<int> values(m_heuristics.size(),0);
    for (size_t i = 0; i < m_heuristics.size(); ++i){
        values[i] = m_heuristics[i]->getGoalHeuristic(state);
    }
    return values;
}

