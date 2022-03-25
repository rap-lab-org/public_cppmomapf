
/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/


#include "dijkstra.hpp"

namespace rzq{
namespace search{

DijkstraScan::DijkstraScan()
{
};

DijkstraScan::~DijkstraScan()
{
};

void DijkstraScan::SetGraphPtr(basic::Graph* g) {
  _graph = g;
};

int DijkstraScan::Search(long src, size_t cdim) {
  
  if ( !_graph->HasNode(src) ) {
    // failed, input src does not exists as a source node in graph.
    throw std::runtime_error( "[ERROR], DijkstraScan input src does not exist !!" );
    return -1;
  } 

  std::set< std::pair<long, long> > open; 
  _v2d.clear();

  open.insert(std::make_pair(0, src));
  _v2d[src] = 0;

  while(!open.empty()){
    std::pair<long, long> curr_pair = *(open.begin());
    open.erase(open.begin());
    auto v = curr_pair.second;
    auto nghs = _graph->GetPreds(v);
    for (auto iter = nghs.begin(); iter != nghs.end(); iter++){
      long u = *iter;
      auto cvec = _graph->GetCost(u,v); // CAVEAT, not the order! (u,v), not (v,u). Because Dijkstra is search backwards !!!
      auto c = cvec[cdim];
      // long c = 1;
      if (c < 0){
        // negative edge !! Input graph has ERROR!!
        std::cout << " v = " << v << " u = " << u << " cost_vec(u,v) = " << cvec << " c = " << c << std::endl;
        throw std::runtime_error( "[ERROR], DijkstraScan encounter negative edge costs !!" );
        _v2d.clear();
        return -2; // failed
      } // end if
      long dist_u = curr_pair.first + c;
      if (_v2d.find(u) == _v2d.end()) {
        // not visited yet.
        _v2d[u] = dist_u;
        open.insert(std::make_pair(dist_u, u));
      }else{
        // already inserted
        if ( dist_u < _v2d[u] ){
          // a better path;
          _v2d[u] = dist_u;
          auto temp_pair = std::make_pair(dist_u,u);
          if ( open.find(temp_pair) != open.end() ) {
            open.erase( open.find(temp_pair) ); // re-insert, delete first.
          }
          open.insert(temp_pair); // re-insert
        }
      }// end else
    }// end for
  }// end while
  return 1;
};
long DijkstraScan::GetCost(long u) {
//   std::cout << " input u = " << u << 
  if (_v2d.find(u) == _v2d.end()) {
    return -1;
  }
  return _v2d[u];
};

std::unordered_map<long, long> DijkstraScan::GetDistMap() {
  return _v2d;
};

} // end namespace search
} // end namespace rzq
