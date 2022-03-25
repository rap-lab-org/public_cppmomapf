
/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/


#ifndef ZHONGQIANGREN_SEARCH_DIJKSTRA_H_
#define ZHONGQIANGREN_SEARCH_DIJKSTRA_H_

#include "graph.hpp"
#include <set>
#include <unordered_map>
#include <limits>

namespace rzq{
namespace search{
/**
 * @brief This is A REVERSE Dijkstra search from the goal to all other nodes.
 * NOTE that this class use GetPreds rather than GetSuccs. This class get cost (u,v) instead of (v,u) with u is a predecessor of v.
 */
class DijkstraScan {
public:
  DijkstraScan();
  virtual ~DijkstraScan();
  virtual void SetGraphPtr(basic::Graph* g) ;
  virtual int Search(long goal, size_t cdim) ;
  /**
   * @brief return the shortest path cost between node u and v.
   * NOTE: return -1 if there is no path between u and v.
   */
  virtual long GetCost(long u) ; 
  virtual std::unordered_map<long, long> GetDistMap() ;
protected:
  basic::Graph* _graph;
  std::unordered_map<long, long> _v2d; // store the results.
};


} // end namespace search
} // end namespace rzq


#endif  // ZHONGQIANGREN_SEARCH_DIJKSTRA_H_
