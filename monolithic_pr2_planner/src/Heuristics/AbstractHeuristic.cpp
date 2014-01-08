#include <monolithic_pr2_planner/Heuristics/AbstractHeuristic.h>

using namespace monolithic_pr2_planner;

AbstractHeuristic::AbstractHeuristic(){ 
	m_cost_multiplier = 1;
}

void AbstractHeuristic::setCostMultiplier(const int cost_multiplier){
	m_cost_multiplier = cost_multiplier;
}

int AbstractHeuristic::getCostMultiplier(){
	return m_cost_multiplier;
}