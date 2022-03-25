
/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/

#ifndef ZHONGQIANGREN_SEARCH_BOA_ST_H_
#define ZHONGQIANGREN_SEARCH_BOA_ST_H_

#include "moa.hpp"
#include "avltree.hpp"

namespace rzq{
namespace search{

#define DEBUG_BOA 0

/**
 * @brief A search label, used by MOA*.
 */
struct LabelST : LabelMOA {
  LabelST() {};
  LabelST(long id0, long v0, basic::CostVector g0, basic::CostVector f0, long t0) {
    id = id0; v = v0; g = g0; f = f0; t = t0;
  };
  long t; // the arrival time at a state.
};

std::ostream& operator<<(std::ostream& os, LabelST& l) ;

/**
 * @brief store all non-dominated vectors. Do linear scan for checking and updating.
 */
class FrontierBOA {
public:
  /**
   * @brief
   */
  FrontierBOA() ;
  virtual ~FrontierBOA() ;
  virtual bool Check(basic::CostVector g) ;
  virtual void Update(LabelST l) ;

  long g2min = std::numeric_limits<long>::max() ;
  std::unordered_set<long> label_ids;
};

std::ostream& operator<<(std::ostream& os, FrontierBOA& f) ;

/**
 * @brief an interface / base class, use its pointer
 */
class BOAst : public MOA{
public:
  BOAst() ;
  virtual ~BOAst() ;

  // // set graph as pointer, note to leverage polymorphism here.
  // virtual void SetGraphPtr(basic::Graph* g) ;

  // // this vd must be the same as the vd in Search().
  // virtual void InitHeu(long vd);

  virtual void SetWaitCost(std::vector<long>& wait_cost) ;

  
  // heuristic computation are included in each search call.
  virtual int Search(long vo, long vd, double time_limit) ;

  // virtual MOAResult GetResult() const ;

  virtual void AddNodeCstr(long nid, long t) ;

  virtual void AddEdgeCstr(long u, long v, long t) ;

  /**
   * @brief a new API for python wrapper. only for grid like world. 
   * Add to make it compatible with pybind11.
   */
  // virtual void SetGrid(basic::GridkConn& g) ;
protected:
  // return the heuristic vector from v to vd.
  // virtual basic::CostVector _Heuristic(long v) ;

  // this method needs to new frontiers, which depend on the specific #obj.
  virtual void _UpdateFrontier(LabelST& l) ;

  virtual void _UpdateSol(LabelST& l) ;

  // virtual long _GenLabelId() ;

  virtual bool _FrontierCheck(LabelST& l) ;
  
  virtual bool _SolutionCheck(LabelST& l) ;

  virtual bool _ReachGoalCondition(LabelST& l) ;

  virtual bool _CollideCheck(long v1, long v2, long t);

  virtual bool _BuildPath(long lid, std::vector<long>* path, std::vector<long>* times) ;

  virtual void _PostProcRes();

  virtual basic::CostVector _GetWaitCost(long v1, long v2, long dt);

  virtual inline std::string _L2S(const LabelST& l) {
    return std::to_string(l.v) + "," + std::to_string(l.t);
  };

  std::unordered_map< std::string, FrontierBOA > _alpha; // map a vertex id (v) to alpha(v).
  FrontierBOA _sols; // solution labels.
  long _last_nc_t = -1;
  std::unordered_map<long, LabelST> _label;
  std::unordered_map<long, basic::AVLTree<long> > _avl_node;
  std::unordered_map<long, std::unordered_map<long, basic::AVLTree<long> > > _avl_edge;
  basic::CostVector _wait_cvec;
    // This can be removed from this algorithm. 
    // BOA*-st does not require const wait vector at all locations.
};


int RunBOAstGrid(
  basic::GridkConn& g, long vo, long vd, double time_limit, basic::CostVector& wait_cost,
  std::vector< std::vector<long> >& ncs, std::vector< std::vector<long> >& ecs, MOSPPResult* res) ;


} // end namespace search
} // end namespace rzq


#endif  // ZHONGQIANGREN_SEARCH_BOA_ST_H_
