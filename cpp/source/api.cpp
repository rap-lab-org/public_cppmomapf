/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/

#include "api.hpp"

namespace rzq{
namespace mapf{


int RunMOCBS(
  basic::GridkConn& g, std::vector<long>& starts, std::vector<long>& goals,
  mapf::MOMAPFResult* res, double time_limit,
  std::vector<long> wait_cost, int ifTreeByTree)
{

  mapf::MOCBS_TB* planner;
  if (g.GetCostDim() == 2) {
    if (ifTreeByTree) {
      planner = new mapf::MOCBS_TB();
    }else{
      planner = new mapf::MOCBS_B();
    }
  }else { // M >= 3
    if (ifTreeByTree) {
      planner = new mapf::MOCBS_TN();
    }else{
      planner = new mapf::MOCBS_N();
    }
  }

  if (g.GetCostDim() == 1) {
    std::cout << "[WARNING] RunMOCBS, You are using MO-CBS to solve a single-objective problem." << std::endl;
    std::cout << "[WARNING] RunMOCBS, MO-CBS-N/TN is created" << std::endl;
  }
  planner->SetGrid(-1, starts.size(), g, wait_cost);
  planner->Search(starts, goals, time_limit, 0.0); 
  *res = planner->GetResult();

  delete planner;

  return 1;
};


}
}