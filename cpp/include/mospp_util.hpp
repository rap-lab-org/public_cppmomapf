
/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/


#ifndef ZHONGQIANGREN_SEARCH_MOSPPUTIL_H_
#define ZHONGQIANGREN_SEARCH_MOSPPUTIL_H_

#include "graph.hpp"

namespace rzq{
namespace search{

/**
 * @brief Epsilon-dominance.
 */
template<typename IterData>
bool EpsDominance(IterData v1, IterData v2, double eps=0.0, bool less=true) {
  auto i2 = v2.begin();
  for (auto i1 = v1.begin(); i1 != v1.end(); i1++){
    if (less) {
      if (*i1 > (1.0+eps)*(*i2)) { return false; }
    }else{
      if (*i1 < (1.0+eps)*(*i2)) { return false; }
    }
    i2++;
  }
  return true;
};

/**
 * @brief result data structure.
 */
struct MOSPPResult {
  bool success = false;
  std::unordered_map< long, std::vector<long> > paths;
  std::unordered_map< long, std::vector<long> > times;
  std::unordered_map< long, basic::CostVector > costs;
  long n_generated = 0;
  long n_expanded = 0;
  double rt_initHeu = 0.0;
  double rt_search = 0.0;
};

/**
 *
 */
std::ostream& operator<<(std::ostream& os, const MOSPPResult& res) ;

} // end namespace search
} // end namespace rzq


#endif  // ZHONGQIANGREN_SEARCH_MOSPPUTIL_H_
