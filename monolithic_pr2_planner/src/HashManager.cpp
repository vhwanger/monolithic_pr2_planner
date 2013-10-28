#include <monolithic_pr2_planner/HashManager.h>
#include <monolithic_pr2_planner/StateReps/DiscObjectState.h>
#include <monolithic_pr2_planner/StateReps/ContArmState.h>
#include <boost/foreach.hpp>
#include <stdexcept>
#include <assert.h>

#define HASH_TABLE_SIZE (32*1024)
using namespace monolithic_pr2_planner;

HashManager::HashManager() : m_coord_to_state_id_table(HASH_TABLE_SIZE){
}

unsigned int HashManager::intHash(unsigned int key){
    key += (key << 12);
    key ^= (key >> 22);
    key += (key << 4); 
    key ^= (key >> 9); 
    key += (key << 10);
    key ^= (key >> 2); 
    key += (key << 7); 
    key ^= (key >> 12);
    return key;
}


// TODO: make sure to add the free angle hash!
unsigned int HashManager::hash(const GraphStatePtr& graph_state){
    int val = 0;
    int counter = 0;
    auto obj_state = graph_state->getDiscObjectState();
    for (auto it=obj_state.getCoordBegin(); it!=obj_state.getCoordEnd();++it){
        val += intHash(*it) << counter;
        counter++;
    }

    auto robot_pose = graph_state->getRobotPose();
    auto base_state = robot_pose.getDiscBaseState();
    for (auto it=base_state.getCoordBegin(); it!=base_state.getCoordEnd();++it){
        val += intHash(*it) << counter;
        counter++;
    }

    val += intHash(robot_pose.getRightDiscFreeAngle()) << counter;
    counter++;

    val += intHash(robot_pose.getLeftDiscFreeAngle()) << counter;
    counter++;

    // should have used 12 values during the hash function
    // xyzrpy(obj pose) fa1 fa2 (arm) xyzyaw(base)

    return val & HASH_TABLE_SIZE;
}

GraphStatePtr HashManager::getGraphState(unsigned int state_id){
    return m_state_id_to_graph_table.at(state_id);
}

unsigned int HashManager::getStateID(const GraphStatePtr& graph_state){
    int index = 0;
    unsigned int bin_idx = hash(graph_state);
    BOOST_FOREACH(auto g_s, m_coord_to_state_id_table.at(bin_idx)){
        if (*g_s == *graph_state){
            return index;
        }
        index++;
    }
    throw std::out_of_range("Graph state does not exist");
}

bool HashManager::exists(const GraphStatePtr& graph_state){
    unsigned int bin_idx = hash(graph_state);
    BOOST_FOREACH(auto g_s, m_coord_to_state_id_table.at(bin_idx)){
        if (*g_s == *graph_state){
            return true;
        }
    }
    return false;
}

bool HashManager::save(GraphStatePtr& graph_state){
    // this may not be the desired behavior...
    if (exists(graph_state)){
        return false;
    }

    unsigned int bin_idx = hash(graph_state);
    m_coord_to_state_id_table[bin_idx].push_back(graph_state);

    graph_state->setID(m_state_id_to_graph_table.size());
    m_state_id_to_graph_table.push_back(graph_state);
    return true;
}


