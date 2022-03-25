/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/

#ifndef ZHONGQIANGREN_MAPF_MOMSTAR_H_
#define ZHONGQIANGREN_MAPF_MOMSTAR_H_

#include "graph.hpp"
#include "momapf_util.hpp"
#include "moa.hpp"

#include <string>
#include <chrono>

namespace rzq{
namespace mapf{

#define DEBUG_MOMSTAR 0
#define MAX_NGH_SIZE 1e7

/**
 * @brief
 */
struct MState
{
  long id = 0;
  std::vector<long> jv; // joint vertex
  basic::CostVector g; // cost vector
  // std::string jv2str() ;
  std::unordered_map<int, int> colSet; // collision set
};

inline std::string jv2str(const std::vector<long>& jv) {
  std::string out("(");
  for (size_t i = 0; i < jv.size(); i++){
    out += std::to_string(jv[i])+",";
  }
  out += ")";
  return out;
};

inline bool jvEqual(const std::vector<long>& jv, const std::vector<long>& jv2) {
  for (size_t i = 0; i < jv.size(); i++){
    if (jv[i] != jv2[i]) {
      return false;
    }
  }
  return true;
}

/**
 * @brief verify whether collision set a is a subset of b.
 * This is subset check as in M*, not as in rM* !
 */
bool IsColSetSubset(
  const std::unordered_map<int,int>& a, const std::unordered_map<int,int>& b) ;

/**
 * @brief take the combination by select an element from every 
 *  sub-vector in the input vec.
 */
template <typename DataType>
void TakeCombination(
  const std::vector< std::vector<DataType> >&, std::vector< std::vector<DataType> >*) ;

/**
 * @brief
 */
class MOMStarPolicy : public search::MOA {
public:
  /**
   *
   */
  MOMStarPolicy() ;
  /**
   *
   */
  virtual ~MOMStarPolicy() ;
  /**
   *
   */
  virtual bool Compute(long vd, double time_limit) ;
  /**
   * @brief for debug
   */
  virtual void Print() ;
  /**
   * @brief get next Pareto-optimal move.
   */
  virtual std::vector<long>  Phi(const long& v) ;
  /**
   * @brief get heuristic vector, component-wise underestimate.
   */
  virtual basic::CostVector H(const long& v) ;
  /**
   * @brief disable
   */
  virtual void InitHeu(long vd) override ;
protected:
  /**
   * @brief disable
   */
  virtual basic::CostVector _Heuristic(long v) override ;
  /**
   * @brief disable
   */
  virtual bool _SolutionCheck(search::LabelMOA l) override ;
  /**
   * @brief Tricky. Get cost for the backward search.
   */
  basic::CostVector _GetEdgeCost(const long& v1, const long& v2) override ;

  std::unordered_map<long, std::unordered_set<long> > _phi;
  std::unordered_map<long, basic::CostVector > _h;

};

/**
 * @brief
 */
class MOMStar : public MAPFPlanner
{
public:
  /**
   *
   */
  MOMStar() ;
  /**
   *
   */
  virtual ~MOMStar() ;
  /**
   * @brief for python usage.
   */
  virtual void SetGrid(size_t n_agents, basic::GridkConn& g, std::vector< std::vector<long> >& wait_costs) ;
  /**
   * @brief 
   */
  virtual void Search(std::vector<long>& starts, std::vector<long>& goals, double time_limit, double wH=1.0) override ;
  /**
   * @brief 
   */
  virtual MOMAPFResult GetResult() const override ;
protected:
  /**
   * @brief 
   */
  virtual bool _GetLimitNgh(const long& sid, std::vector< std::vector<long> >* out);
  /**
   * @brief 
   */
  inline long _GenId() { return _id_gen++; };
  /**
   * @brief. Return false if timeout.
   */
  virtual bool _Init() ;
  /**
   * @brief 
   */
  virtual basic::CostVector _H(const std::vector<long>& jv) ;
  /**
   * @brief 
   */
  virtual bool _IfReachGoal(const std::vector<long>& jv) ;
  /**
   * @brief 
   */
  std::unordered_map<int,int> _ColCheck(
    const std::vector<long>& jv1, const std::vector<long>& jv2) ;
  /**
   * @brief Add sid2 to sid1.back_set
   */
  virtual void _AddBackSet(const long& sid1, const long& sid2) ;
  /**
   * @brief Recursive backprop of collision set.
   */
  virtual void _BackProp(const long& sid, const std::unordered_map<int,int>& col_set) ;
  /**
   * @brief union the collision set.
   */
  virtual void _ColSetUnion(
    const std::unordered_map<int,int>& a, std::unordered_map<int,int>* b) ;
  /**
   * @brief reopen a state.
   */
  virtual void _Reopen(const long& sid) ;
  /**
   * @brief get transition cost vector, summed over all agents.
   */
  virtual basic::CostVector _GetTransCost(
    const std::vector<long>& jv1, const std::vector<long>& jv2, long curr_sid) ;
  /**
   * @brief Compare whether a newly generate state has a cost vector that is dominated by some solution found.
   */
  virtual bool _SolFilter(const basic::CostVector&);
  /**
   * @brief Compare whether a newly generate state should be pruned or not.
   */
  virtual bool _DomCompare(const std::vector<long>&, const basic::CostVector&, std::vector<long>*) ; 
  /**
   *
   */
  virtual void _PostProcessResult() ;
  /**
   *
   */
  virtual void _BuildJointPath(long sid, std::vector< std::vector<long> >* jp) ;
  /**
   *
   */
  virtual void _JPath2PathSet(const std::vector< std::vector<long> >& jp, PathSet* ps) ;
  /**
   * @brief. Compute the cost of a special case where robot stays at the goal for a while and then moves away.
   */
  virtual basic::CostVector _MoveFromGoalCost(const int& ri, long sid) ;

  size_t _nAgent = 0;
  std::chrono::time_point<std::chrono::steady_clock> _t0;
  double _tlimit; // the allowed time for planning.
  std::vector<long> _vo;
  std::vector<long> _vd;
  basic::Graph* _g;
  std::unordered_map<int, MOMStarPolicy> _policies;
  long _id_gen = 1;
  std::unordered_map<long, MState> _states;
  std::unordered_map<long,long> _parent; // map a state id to its parent state id.
  std::unordered_map<long, std::unordered_set<long> > back_set_map_;
  std::set< std::pair< basic::CostVector, long> > _open; // <cost vector, state id>
  std::unordered_map< std::string, std::unordered_set<long> > _frontiers; 
  std::vector<long> _sol;
  std::unordered_map<int, basic::CostVector> _agentCosts;
  MOMAPFResult _result;
  double _wH = 1.0;

};

} // end namespace mapf
} // end namespace rzq

#endif  // ZHONGQIANGREN_MAPF_MOMSTAR_H_
