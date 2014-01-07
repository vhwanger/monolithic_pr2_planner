#include <monolithic_pr2_planner/Heuristic.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <monolithic_pr2_planner/Visualizer.h>

using namespace monolithic_pr2_planner;
using namespace sbpl_arm_planner;
using namespace std;

Heuristic::Heuristic(){ 
    int dimX, dimY, dimZ;
    m_occupancy_grid->getGridSize(dimX, dimY, dimZ);
    m_bfs.reset(new BFS_3D(dimX, dimY, dimZ));
}

int Heuristic::getGoalHeuristic(GraphStatePtr state){
    DiscObjectState obj_state = state->getObjectStateRelMap();
    int cost = m_bfs->getDistance(obj_state.x(), obj_state.y(), obj_state.z());
    ROS_DEBUG_NAMED(SEARCH_LOG, "dijkstra's cost to %d %d %d is %d", 
                    obj_state.x(), obj_state.y(), obj_state.z(), cost);
    return cost;
}

void Heuristic::setGoal(GoalState& goal_state){
    DiscObjectState state = goal_state.getObjectState(); 
    m_bfs->run(state.x(),
               state.y(),
               state.z());
    ROS_DEBUG_NAMED(CONFIG_LOG, "running heuristic on new goal %d %d %d",
                    state.x(), state.y(), state.z());
}

void Heuristic::loadObstaclesFromOccupGrid(){
    int dimX, dimY, dimZ;
    m_occupancy_grid->getGridSize(dimX, dimY, dimZ);
    int walls = 0;
    for (int z = 0; z < dimZ - 2; z++){
        for (int y = 0; y < dimY - 2; y++){
            for (int x = 0; x < dimX - 2; x++){
                if(m_occupancy_grid->getDistance(x,y,z) <= m_resolution_params.gripper_sphere_radius){
                    m_bfs->setWall(x + 1, y + 1, z + 1); //, true);
                    walls++;
                }
            }
        }
    }
    ROS_DEBUG_NAMED(CONFIG_LOG, "Initialized heuristic with %d walls", walls);
    ROS_DEBUG_NAMED(CONFIG_LOG, "using gripper sphere radius %f", 
                    m_resolution_params.gripper_sphere_radius);
}

void Heuristic::visualize(){
    int dimX, dimY, dimZ;
    m_occupancy_grid->getGridSize(dimX, dimY, dimZ);
    vector<vector<double> > obstacles; 
    for (int z = 0; z < dimZ - 2; z++){
        for (int y = 0; y < dimY - 2; y++){
            for (int x = 0; x < dimX - 2; x++){
                if(m_occupancy_grid->getDistance(x,y,z) <= m_resolution_params.gripper_sphere_radius){
                    vector<double> point;
                    point.push_back(x+1);
                    point.push_back(y+1);
                    point.push_back(z+1);
                    point.push_back(.1);
                    point.push_back(.1);
                    point.push_back(.1);

                    obstacles.push_back(point);
                }
            }
        }
    }
    Visualizer::pviz->visualizeObstacles(obstacles);
}
