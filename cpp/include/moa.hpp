
/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/

/*******************************************************************************************
 * A modern implementation of NAMOA*.
 *******************************************************************************************/

#ifndef ZHONGQIANGREN_SEARCH_MOA_H_
#define ZHONGQIANGREN_SEARCH_MOA_H_

#include "graph.hpp"
#include "dijkstra.hpp"
#include "mospp_util.hpp"

#include <limits>
#include <list>
#include <set>

namespace rzq{
namespace search{

#define DEBUG_MOA 0

/**
 * @brief A search label, used by MOA*.
 */
struct LabelMOA {
  LabelMOA() {};
  LabelMOA(long id0, long v0, basic::CostVector g0, basic::CostVector f0) {
    id = id0; v = v0; g = g0; f = f0;
  };
  long id; // label's id, make it easy to look up.
  long v;
  basic::CostVector g;
  basic::CostVector f;
};

std::ostream& operator<<(std::ostream& os, LabelMOA& l) ;


/**
 * @brief store all non-dominated vectors. Do linear scan for checking and updating.
 */
class FrontierNaive {
public:
  /**
   * @brief
   */
  FrontierNaive() ;
  virtual ~FrontierNaive() ;
  virtual bool Check(basic::CostVector g) ;
  virtual void Update(LabelMOA l) ;
// protected: // make ostream easier.
  std::list<basic::CostVector> _nondom;
  std::unordered_set<long> label_ids;
};

std::ostream& operator<<(std::ostream& os, FrontierNaive& f) ;

/**
 * @brief an interface / base class
 */
class MOA {
public:
  MOA() ;
  virtual ~MOA() ;

  // set graph as pointer, note to leverage polymorphism here.
  virtual void SetGraphPtr(basic::Graph* g) ;

  // this vd must be the same as the vd in Search().
  virtual void InitHeu(long vd);
  
  // heuristic computation are included in each search call.
  virtual int Search(long vo, long vd, double time_limit) ;

  virtual MOSPPResult GetResult() const ;

  /**
   * @brief This is only for grid like world. 
   */
  virtual void SetGrid(basic::GridkConn& g) ;

protected:
  // return the heuristic vector from v to vd.
  virtual basic::CostVector _Heuristic(long v) ;

  // this method needs to new frontiers, which depend on the specific #obj.
  virtual void _UpdateFrontier(LabelMOA l) ;

  virtual long _GenLabelId() ;

  virtual bool _FrontierCheck(LabelMOA l) ;
  
  virtual bool _SolutionCheck(LabelMOA l) ;

  virtual basic::CostVector _GetEdgeCost(const long& u, const long& v);

  virtual bool _BuildPath(long lid, std::vector<long>* path, std::vector<long>* times) ;

  basic::Graph* _graph;
  std::unordered_map< long, FrontierNaive* > _alpha; // map a vertex id (v) to alpha(v).
  long _label_id_gen = 0;
  long _vo = -1, _vd = -1;
  std::set< std::pair< basic::CostVector, long> > _open;
  std::unordered_map<long, LabelMOA> _label;
  std::unordered_map<long, long> _parent;
  MOSPPResult _res;
  std::vector<DijkstraScan> _dijks;

  basic::GridkConn temp_g;
};


} // end namespace search
} // end namespace rzq


#endif  // ZHONGQIANGREN_SEARCH_MOA_H_
