#include <monolithic_pr2_planner/HashManager.h>
#include <monolithic_pr2_planner/StateReps/DiscObjectState.h>
#include <monolithic_pr2_planner/StateReps/ContArmState.h>
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
unsigned int HashManager::hash(const GraphState& graph_state){
    int val = 0;
    int counter = 0;
    auto obj_state = graph_state.getDiscObjectState();
    for (auto it=obj_state.getCoordBegin(); it!=obj_state.getCoordEnd();++it){
        val += intHash(*it) << counter;
        counter++;
    }

    auto robot_pose = graph_state.getRobotPose();
    auto base_state = robot_pose.getBaseState();
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
    assert(counter==13);

    return val & HASH_TABLE_SIZE;
}
