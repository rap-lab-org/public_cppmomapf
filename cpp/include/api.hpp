/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/

#ifndef ZHONGQIANGREN_BASIC_API_H_
#define ZHONGQIANGREN_BASIC_API_H_

#include "mocbs.hpp"

namespace rzq{
namespace mapf{

/**
 * 
 */
int RunMOCBS(
  basic::GridkConn& g, std::vector<long>& starts, std::vector<long>& goals,
  mapf::MOMAPFResult* res, double time_limit=300,
  std::vector<long> wait_cost=std::vector<long>(), int ifTreeByTree=true) ;

}
}

#endif  // ZHONGQIANGREN_BASIC_API_H_
