/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/

#ifndef ZHONGQIANGREN_MAPF_MOCBS_H_
#define ZHONGQIANGREN_MAPF_MOCBS_H_

#include "graph.hpp"
#include "momapf_util.hpp"
#include "boa_st.hpp"
#include "namoa_dr_st.hpp"

#include <chrono>

namespace rzq{
namespace mapf{

#define DEBUG_MOCBS 0

/**
 *
 */
struct CBSConstraint {
  int i = -1;
  int j = -1;
  long vi = -1;
  long vj = -1;
  long ta = -1;
  long tb = -1;
  CBSConstraint() ; 
  CBSConstraint(int, int, long, long, long, long) ;
  inline bool IsNodeCstr() { return (vi == vj); } ;
  inline bool IsEdgeCstr() { return (vi != vj); } ;
  inline bool IsValid() {return !(ta == -1); } ;
};
/**
 *
 */
std::ostream& operator<<(std::ostream& os, const CBSConstraint& c) ;
/**
 *
 */
struct CBSConflict {
  int i = -1;
  int j = -1;
  long vi = -1;
  long vj = -1;
  long ta = -1;
  long tb = -1;
  std::vector<CBSConstraint> Split();
  inline bool IsValid() {return !(ta == -1); };
};
/**
 *
 */
std::ostream& operator<<(std::ostream& os, const CBSConflict& c) ;
/**
 * @brief Find the first conflict along the paths.
 * The input paths must be unit time paths.
 */
CBSConflict DetectConflict(std::vector<long>* p1, std::vector<long>* p2) ;
/**
 * @brief Find the first conflict along the paths.
 * The input paths within the path set must be unit time paths.
 */
CBSConflict DetectConflict(PathSet* p) ;

/**
 *
 */
bool UnitTimePath(std::vector<long>& p0, std::vector<long> t0,
  std::vector<long>* p1, std::vector<long>* t1, long dt=1) ;

/**
 *
 */
struct CBSNode {
  long id = -1; // ID of this node.
  long parent = -1; // parent node ID.
  PathSet paths; // a set of individual paths.
  std::unordered_map<int, basic::CostVector> costs;
  basic::CostVector g;
  CBSConstraint cstr;
};
/**
 *
 */
std::ostream& operator<<(std::ostream& os, const CBSNode& n) ;

/**
 *
 */
class MOCBS_TB : public MAPFPlanner
{
public:
  /**
   *
   */
  MOCBS_TB();
  /**
   *
   */
  virtual ~MOCBS_TB();
  /**
   * @brief only for grid like world now.
   */
  virtual void SetGrid(int ri, size_t n_agents, basic::GridkConn& g, std::vector<long>& wait_cost) ;
  /**
   * @brief 
   */
  virtual void SetWaitCosts(std::vector<basic::CostVector>& wait_costs) ;
  /**
   * @brief epsilon here is used as epsilon dominance.
   */
  virtual void Search(std::vector<long>& starts, std::vector<long>& goals, double time_limit, double eps) override ;
  /**
   *
   */
  virtual MOMAPFResult GetResult() const override ;
protected:
  /**
   *
   */
  virtual bool _Init();
  /**
   *
   */
  virtual CBSNode* _SelectNode();
  /**
   * @brief Generate a new root node, add to OPEN, return a pointer to this node via *out.
   */
  virtual CBSNode* _NextRoot() ;
  /**
   *
   */
  virtual bool __NextRootUpdateIndex() ;
  /**
   *
   */
  virtual bool _IfTimeout() ;
  /**
   * @brief check for dominance between (cost vector g) and solutions that are already found.
   */
  virtual bool _SolutionFilter(basic::CostVector& g) ;
  /**
   * @brief Placeholder here. Not in effect. always return false (non-dominated, non-filtered).
   */
  virtual bool _SuccessorFilter(CBSNode* n, basic::CostVector& g);
  /**
   * @brief node *in contains a conflict-free solution, use *in to update solution set.
   */
  virtual void _UpdateSolution(CBSNode* n) ;
  /**
   *
   */
  virtual bool _GetAllCstr(int ri, CBSNode* n, std::vector<CBSConstraint*> *out) ;
  /**
   * @brief given a robot id (arg ri), and a set of constraints (arg cstrs) on agent-ri, 
   *   compute a set of individual Pareto-optimal paths, times, costs, etc. (passed out 
   *   via arg *out). 
   *   The returned bool flag indicates whether this low level plan succeeds or not.
   */
  virtual int _LsearchPlan(int ri, std::vector<CBSConstraint*>& cstrs, search::MOSPPResult* out) ;
  virtual int _Lsearch(int ri, CBSNode* n, CBSConstraint* cp) ;
  /**
   *
   */
  virtual inline long _GenNodeId() { return _idGen++; };

  /**
   * @brief This method is invoked after init Pareto-optimal paths of each agent is computed and stored in _initRes.
   * This method will _initPathId and _initPathIndex, which will be used by _GetPathSetGetCurr
   */
  virtual void __InitPathSetUp(int ri) ;

  virtual void __InitPathGetCurr(
    int ri, std::vector<long>* path, std::vector<long>* time, basic::CostVector *g) ;

  virtual void _GetPathSetGetCurr(CBSNode* n) ;
  /**
   * @brief not a public API! will be invoked at the beginning of Search().
   */
  virtual void SetGraphPtr(int ri, size_t n_agents, basic::Graph* g, std::vector<long>& wait_cost) ;

  // Problem related info
  std::unordered_map<int, basic::CostVector> _agentCosts;
  std::unordered_map<int, basic::Graph*> _agentGraphs; // each agent may have different cost grids.
  std::unordered_map<int, basic::GridkConn> _tempAgentGrids; //
  size_t _cdim = 0;
  std::vector<long> _vo; // joint start
  std::vector<long> _vd; // joint destination
  size_t _nAgent = 0;

  // Init Root related
  std::unordered_map<int, search::MOSPPResult> _initRes; // init paths of all agents
  std::unordered_map<int, std::vector<long> > _initPathId; // 
  std::unordered_map<int, size_t> _initPathIndex; // 

  // Search related
  long _idGen = 0;
  std::chrono::time_point<std::chrono::steady_clock> _t0;
  double _tlimit; // the allowed time for planning.
  std::unordered_map<long, CBSNode> _nodes; // all nodes generated during the search.
  std::set< std::pair< basic::CostVector, long> > _open;
  MOMAPFResult _result;

  double _epsilon = 0.0;
};

/**
 * @brief the most basic version, without tree-by-tree.
 * Use MOA*-st as the low level search.
 */
class MOCBS_TN : public MOCBS_TB {
public:
  MOCBS_TN() ;
  virtual ~MOCBS_TN() ;
protected:
  virtual int _LsearchPlan(int ri, std::vector<CBSConstraint*>& cstrs, search::MOSPPResult* out) override;
};

/**
 * @brief the most basic version, without tree-by-tree.
 * Use BOA*-st as the low level search.
 */
class MOCBS_B : public MOCBS_TB {
public:
  MOCBS_B() ;
  virtual ~MOCBS_B() ;
protected:
  /**
   *
   */
  virtual bool _Init() override ;
  virtual CBSNode* _SelectNode() override ;
};

/**
 * @brief the most basic version, without tree-by-tree.
 * Use MOA*-st as the low level search.
 */
class MOCBS_N : public MOCBS_B {
public:
  MOCBS_N() ;
  virtual ~MOCBS_N() ;
protected:
  virtual int _LsearchPlan(int ri, std::vector<CBSConstraint*>& cstrs, search::MOSPPResult* out) override;
};



} // end namespace mapf
} // end namespace rzq

#endif  // ZHONGQIANGREN_MAPF_MOCBS_H_
