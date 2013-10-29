#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <vector>
#include <boost/shared_ptr.hpp>

namespace monolithic_pr2_planner {
    class HashManager {
        public:
            HashManager();
            GraphStatePtr getGraphState(unsigned int state_id);
            unsigned int getStateID(const GraphStatePtr& graph_state);
            bool exists(const GraphStatePtr& graph_state);
            bool save(GraphStatePtr& graph_state);

        private:
            unsigned int hash(const GraphStatePtr& graph_state);
            unsigned int intHash(unsigned int val);
            std::vector<GraphStatePtr> m_state_id_to_graph_table;
            std::vector<std::vector<GraphStatePtr> > m_coord_to_state_id_table;
    };
}
