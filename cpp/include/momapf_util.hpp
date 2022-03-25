
/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/


#ifndef ZHONGQIANGREN_SEARCH_MOMAPFUTIL_H_
#define ZHONGQIANGREN_SEARCH_MOMAPFUTIL_H_

// #include <unordered_map>
// #include <vector>
// #include <iostream>
#include "graph.hpp"

namespace rzq{
namespace mapf{

/**
 *
 */
typedef std::unordered_map<int, std::vector<long>> PathSet ;
/**
 *
 */
std::ostream& operator<<(std::ostream& os, const PathSet& ps) ;
/**
 *
 */
struct MOMAPFResult
{
  bool find_all_pareto = false;
  std::unordered_map< long, PathSet > solus; // each solution is a PathSet (like joint path) !
  std::unordered_map< long, basic::CostVector > costs; // the cost vector corresponding to each PathSet !
  long n_generated = 0;
  long n_expanded = 0; // #conflicts resolved in MO-CBS, or #expansions in MOM*.
  long n_initRoot = 0; // for MO-CBS only
  long n_maxColSet = 0; // for MOM* only
  long n_solFiltered = 0; // for both
  double n_branch = 0.0; // for both MO-CBS and MOM*. average branching factor per expansion.
  double rt_initHeu = 0.0; // in MO-CBS non-tree-by-tree version, this stores init all root time.
  double rt_search = 0.0;
  double rt_searchLow = 0.0; // for MO-CBS only
  double n_expandLow = 0.0; // for MO-CBS only, total number of low level state expanded.
  double rt_firstSol = 0.0;
  basic::CostVector firstSolCost; // not used in MOM*
};

std::ostream& operator<<(std::ostream& os, const MOMAPFResult& res) ;

/**
 * @brief
 */
class MAPFPlanner {
public:
  /**
   * @brief
   */
  MAPFPlanner() {} ;
  /**
   * @brief
   */
  virtual ~MAPFPlanner() {} ;
  /**
   * @brief
   */
  virtual void Search(std::vector<long>& starts, std::vector<long>& goals, double time_limit, double eps) = 0;
  /**
   * @brief
   */
  virtual MOMAPFResult GetResult() const = 0;
};

} // end namespace mapf
} // end namespace rzq


#endif  // ZHONGQIANGREN_SEARCH_MOMAPFUTIL_H_
