#pragma once
#include <pviz/pviz.h>
#include <string>
#include <memory>

namespace monolithic_pr2_planner {
    class Visualizer {
        public:
            static void setReferenceFrame(std::string frame);
            static void createPVizInstance();
            static std::unique_ptr<PViz> pviz;
    };
}
