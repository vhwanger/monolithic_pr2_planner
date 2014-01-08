#include <monolithic_pr2_planner/BaseHeuristic.h>
#include <monolithic_pr2_planner/LoggerNames.h>
using namespace monolithic_pr2_planner;

BaseHeuristic::BaseHeuristic(){
    int threshold = 255;
    int dimX, dimY, dimZ;
    m_occupancy_grid->getGridSize(dimX, dimY, dimZ);
    m_size_col = static_cast<unsigned int>(dimX);
    m_size_row = static_cast<unsigned int>(dimY);
    m_bfs.reset(new sbpl_bfs_2d(m_size_col, m_size_row, threshold));
}

BaseHeuristic::~BaseHeuristic(){
    for (unsigned int i=0; i < m_size_col; i++){
        delete m_grid[i];
    }
}

void BaseHeuristic::loadMap(std::vector<signed char>& data){
    int** m_grid = new int*[m_size_col];
    for (unsigned int i=0; i < m_size_col; i++){
        m_grid[i] = new int[m_size_row];
        for (unsigned int j=0; j < m_size_row; j++){
            m_grid[i][j] = data[i*m_size_row+j];
        }
    }
    ROS_DEBUG_NAMED(CONFIG_LOG, "loaded in base heuristic map of size %lu",
                    data.size());
}

void BaseHeuristic::setGoal(DiscObjectState& state){
    m_bfs->compute_distance_from_point(m_grid, state.x(), state.y());
}
