#pragma once
#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <vector>
#include <boost/shared_ptr.hpp>

namespace monolithic_pr2_planner {
    /*! \brief Contains functions for creating and getting graph states used by
     * the environment and SBPL planner.
     */
    class HashManager {
        public:
            HashManager(std::vector<int*>* stateID2Mapping);
            GraphStatePtr getGraphState(int state_id);
            unsigned int getStateID(const GraphStatePtr& graph_state);
            bool exists(const GraphStatePtr& graph_state, int& potential_id);
            bool save(GraphStatePtr& graph_state);
            int size() { return static_cast<int>(m_state_id_to_graph_table.size()); } ;

        private:
            // TODO yuck pointer to stack variable
            std::vector<int*>* m_stateID2Mapping;
            unsigned int hash(const GraphStatePtr& graph_state);
            unsigned int intHash(unsigned int val);
            std::vector<GraphStatePtr> m_state_id_to_graph_table;
            std::vector<std::vector<GraphStatePtr> > m_coord_to_state_id_table;
    };
    typedef boost::shared_ptr<HashManager> HashManagerPtr;
}
