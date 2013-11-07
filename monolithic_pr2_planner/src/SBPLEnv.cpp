#include <monolithic_pr2_planner/SBPLEnv.h>

using namespace monolithic_pr2_planner;

SBPLEnv::SBPLEnv(ros::NodeHandle nh) : Environment(nh){}

bool SBPLEnv::InitializeMDPCfg(MDPConfig *MDPCfg) {
  //MDPCfg->goalstateid = envMobileArm.goalHashEntry->stateID;
  //MDPCfg->startstateid = envMobileArm.startHashEntry->stateID;
  return true;
}

int SBPLEnv::SizeofCreatedEnv() {
    return m_hash_mgr.size();
}

void SBPLEnv::PrintState(int stateID, bool bVerbose, FILE* fOut /*=NULL*/) {

}

void SBPLEnv::GetSuccs(int SourceStateID, vector<int>* SuccIDV, vector<int>* CostV) {
  //GetSuccs(SourceStateID,SuccIDV,CostV,NULL);
}
