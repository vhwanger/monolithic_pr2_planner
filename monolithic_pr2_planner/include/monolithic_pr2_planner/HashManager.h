#include <monolithic_pr2_planner/StateReps/GraphState.h>
#include <vector>
#include <boost/shared_ptr.hpp>

namespace monolithic_pr2_planner {
    typedef boost::shared_ptr<GraphState> GraphStatePtr;
    class HashManager {
        public:
            HashManager();
            GraphStatePtr get(unsigned int state_id);
            bool exists(const GraphState& graph_state);
            bool save(const GraphState& graph_state);

        private:
            unsigned int hash(const GraphState& graph_state);
            unsigned int intHash(unsigned int val);
            std::vector<GraphStatePtr> m_state_id_to_graph_table;
            std::vector<std::vector<GraphStatePtr> > m_coord_to_state_id_table;
    };
}
