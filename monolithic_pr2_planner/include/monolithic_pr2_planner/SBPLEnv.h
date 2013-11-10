#pragma once
#include <sbpl/headers.h>
#include <monolithic_pr2_planner/Environment.h>
#include <ros/ros.h>

namespace monolithic_pr2_planner {
    class SBPLEnv : public DiscreteSpaceInformation, public Environment {
        public:
            SBPLEnv(ros::NodeHandle nh);
            bool InitializeEnv(const char* sEnvFile){return false;};
            bool InitializeMDPCfg(MDPConfig *MDPCfg);
            int  GetFromToHeuristic(int FromStateID, int ToStateID){ return -1; };
            int  GetGoalHeuristic(int stateID) { return -1; };
            int  GetStartHeuristic(int stateID) { return -1; };
            void GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV);
            void GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV){};
            void SetAllActionsandAllOutcomes(CMDPSTATE* state){};
            void SetAllPreds(CMDPSTATE* state){};
            int  SizeofCreatedEnv();
            void PrintState(int stateID, bool bVerbose, FILE* fOut=NULL);
            void PrintEnv_Config(FILE* fOut){};
            
    };
}
