#include <monolithic_pr2_planner/Heuristics/BFS2DHeuristic.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <monolithic_pr2_planner/Visualizer.h>
#include <sbpl/utils/key.h>
using namespace monolithic_pr2_planner;

BFS2DHeuristic::BFS2DHeuristic(){
    int threshold = 100;
    int dimX, dimY, dimZ;
    m_occupancy_grid->getGridSize(dimX, dimY, dimZ);
    m_size_col = static_cast<unsigned int>(dimX);
    m_size_row = static_cast<unsigned int>(dimY);
    ROS_DEBUG_NAMED(HEUR_LOG, "[BFS2D] initialized BFS2D of size %d %d", 
                              m_size_col, m_size_row);
    m_bfs.reset(new sbpl_bfs_2d(m_size_col, m_size_row, threshold));
    
    // Initialize the grid here itself so that you don't wait for the map
    // callback to be called
    m_grid = new int*[m_size_col];
    for (unsigned int i=0; i < m_size_col; i++){
        m_grid[i] = new int[m_size_row];
        for (unsigned int j=0; j < m_size_row; j++){
            m_grid[i][j] = 0;
        }
    }
    m_costmap_ros.reset(new costmap_2d::Costmap2DROS("costmap_2d", m_tf));
}

// BFS2DHeuristic::BFS2DHeuristic(boost::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros):
//     m_costmap_ros(costmap_ros) {
//     BFS2DHeuristic();
// }

BFS2DHeuristic::~BFS2DHeuristic(){
    for (unsigned int i=0; i < m_size_col; i++){
        delete m_grid[i];
    }
}

void BFS2DHeuristic::update2DHeuristicMap(const std::vector<signed char>& data){
    loadMap(data);
}

void BFS2DHeuristic::loadMap(const std::vector<signed char>& data){
    sleep(5);
    costmap_2d::Costmap2D cost_map;
    m_costmap_ros->getCostmapCopy(cost_map);

    for (unsigned int i=0; i < m_size_col; i++){
        for (unsigned int j=0; j < m_size_row; j++){
            m_grid[i][j] = (data[i*m_size_row+j]==-1)?0:data[i*m_size_row+j];
            // ROS_DEBUG_NAMED(HEUR_LOG, "[costmap] %d %d : %d", i, j,
            //     cost_map.getCost(i,j));
        }
    }
    

    ROS_DEBUG_NAMED(HEUR_LOG, "[BFS2D] updated grid of size %d %d from the map", m_size_col, m_size_row);
    // visualize();
}

void BFS2DHeuristic::setGoal(GoalState& goal_state){
    DiscObjectState state = goal_state.getObjectState(); 
    m_bfs->compute_distance_from_point(m_grid, state.x(), state.y());
    ROS_DEBUG_NAMED(HEUR_LOG, "[BFS2D] Setting goal %d %d", state.x(), state.y());
}

int BFS2DHeuristic::getGoalHeuristic(GraphStatePtr state){
    DiscObjectState obj_state = state->getObjectStateRelMap();
    
    // Check if within bounds. We need to do this here because the bfs2d
    // implementation doesn't take care of this.
    if (state->base_x() < 0 || state->base_x() >= int(m_size_col) || 
        state->base_y() < 0 || state->base_y() >= int(m_size_row)) {
        ROS_DEBUG_NAMED(HEUR_LOG, "[BFS2D] Out of bounds: %d %d", 
                                  state->base_x(), state->base_y());
        return INFINITECOST;
    }
    int cost = m_bfs->get_distance(state->base_x(), state->base_y());
    ROS_DEBUG_NAMED(HEUR_LOG, "[BFS2D] 2Ddijkstra's cost to %d %d is %d", 
                              state->base_x(), state->base_y(),
                              (getCostMultiplier()*cost)<0?INFINITECOST:getCostMultiplier()*cost );
    if (cost < 0){
        return INFINITECOST;
    }
    return getCostMultiplier()*cost;
}

void BFS2DHeuristic::visualize(){
    int dimX, dimY, dimZ;
    int threshold = 50;
    m_occupancy_grid->getGridSize(dimX, dimY, dimZ);
    vector<vector<double> > obstacles; 
    for (int z = 0; z < dimZ - 2; z++){
        for (int y = 0; y < dimY - 2; y++){
            for (int x = 0; x < dimX - 2; x++){
                if(m_grid[x][y] >= threshold){
                    vector<double> point;
                    point.push_back(x+1);
                    point.push_back(y+1);
                    point.push_back(0.1);
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
