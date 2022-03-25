
/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/

#include "boa_st.hpp"

#include <set>
#include <memory>
#include <chrono>

namespace rzq{
namespace search{

std::ostream& operator<<(std::ostream& os, LabelST& l)
{
  std::string s;
  s = "{id:" + std::to_string(l.id) + ",v:" + std::to_string(l.v) + ",g:" 
    + l.g.ToStr() + ",f:" + l.f.ToStr() + ",t:" + std::to_string(l.t) + "}";
  os << s;
  return os;
};

std::ostream& operator<<(std::ostream& os, FrontierBOA& f) {
  os << "{g2min:" << f.g2min << "}";
  return os;
};

FrontierBOA::FrontierBOA() {
  return;
};

FrontierBOA::~FrontierBOA() {
  return;
};

bool FrontierBOA::Check(basic::CostVector g) {
  if (g2min <= g[1]) return true;
  return false;
};

void FrontierBOA::Update(LabelST l) {
  g2min = l.g[1];
  this->label_ids.insert(l.id);
  return ;
};


BOAst::BOAst() {};

BOAst::~BOAst() {};


void BOAst::SetWaitCost(std::vector<long>& wait_cost) {
  _wait_cvec.resize(wait_cost.size());
  for (size_t ii = 0; ii < wait_cost.size(); ii++){
    _wait_cvec[ii] = wait_cost[ii];
  }
  return;
};

int BOAst::Search(long vo, long vd, double time_limit) {
  if (DEBUG_BOA > 0) {
    std::cout << "[DEBUG] enter BOAst Search " << std::endl;
  }
  // ### init heu ###
  InitHeu(vd);

  // ### init ###
  auto tstart = std::chrono::steady_clock::now();
  _vo = vo;
  _vd = vd;
  basic::CostVector zero_vec;
  zero_vec.resize(_graph->GetCostDim(), 0);
  LabelST lo(_GenLabelId(), vo, zero_vec, _Heuristic(_vo), 0); // t = 0 
  _label[lo.id] = lo;
  _res.n_generated++;
  // _UpdateFrontier(lo);
  _open.insert( std::make_pair(lo.f, lo.id) );

  if (DEBUG_BOA > 0) {
    std::cout << "[DEBUG] Init, lo = " << lo << std::endl;
  }

  // ### main search loop ###
  while ( !_open.empty() ) {
    // ## select label l, lexicographic order ##
    LabelST l = _label[ _open.begin()->second ];
    _open.erase(_open.begin());

    if (DEBUG_BOA > 0) {
      std::cout << "[DEBUG] ### Pop l = " << l << std::endl;
    }

    auto tnow = std::chrono::steady_clock::now();
    if ( std::chrono::duration<double>(tnow-tstart).count() > time_limit) {
      return 0; // fails. timeout.
    }

    // ## lazy dominance check ##
    if ( _FrontierCheck(l) || _SolutionCheck(l) ) {
      if (DEBUG_BOA > 1) {
        std::cout << "[DEBUG] F- AND S-check, dom, cont..." << std::endl;
      }
      continue;
    }
    if ( _ReachGoalCondition(l) ) {
      _UpdateSol(l); // a standalone solution set.
    }
    _UpdateFrontier(l);
    if (DEBUG_BOA > 1) {
      std::cout << "[DEBUG] ### Exp. " << l << std::endl;
    }

    // ## expand label l ##
    _res.n_expanded++;
    auto succs = _graph->GetSuccs(l.v);
    succs.insert(l.v); // stay in place action.
    for (auto u : succs) {
      // move from (v,t) to (u,t+1), collision check
      if ( _CollideCheck(l.v, u, l.t) ) {
        continue; // this ngh (u,t) is in conflict, skip.
      }
      basic::CostVector gu = l.g + _GetEdgeCost(l.v, u);
      if (u == l.v) { // wait in place action.
        gu = l.g + _GetWaitCost(l.v, u, 1);
      }
      LabelST l2(_GenLabelId(), u, gu, gu + _Heuristic(u), l.t+1); // unit time move
      _label[l2.id] = l2;
      _parent[l2.id] = l.id;
      
      if (DEBUG_BOA > 0) {
        std::cout << "[DEBUG] >>>> Loop v= " << u << " gen l' = " << l2 << std::endl;
      }
      if (_FrontierCheck(l2)) {
        if (DEBUG_BOA > 1) {
          std::cout << "[DEBUG] ----- F-Check, dom, cont..." << std::endl;
        }
        continue;
      }
      if (DEBUG_BOA > 0) {
        std::cout << "[DEBUG] ----- Add to open..." << std::endl;
      }
      _res.n_generated++;
      _open.insert( std::make_pair(l2.f, l2.id) );
    }
  };

  // ### post-process the results ###
  _PostProcRes();

  auto tend = std::chrono::steady_clock::now();
  _res.rt_search = std::chrono::duration<double>(tend-tstart).count();
  return 1;
};

void BOAst::_PostProcRes() {
  for (auto lid : _sols.label_ids) {
    _res.paths[lid] = std::vector<long>();
    _res.times[lid] = std::vector<long>();
    bool ret_flag = _BuildPath(lid, &(_res.paths[lid]), &(_res.times[lid]) );
      // when this flag is used, remember to check here for safety.
    _res.costs[lid] = _label[lid].g;

    if ( long(_res.paths[lid].size()) <= _last_nc_t) {
      std::cout << "[ERROR] path length < _last_nc_t !!! " << " size = " 
        << _res.paths[lid].size() << " _last_nc_t = " << _last_nc_t << std::endl;
      throw std::runtime_error("[ERROR]");
    }
  }
};

bool BOAst::_BuildPath(long lid, std::vector<long>* path, std::vector<long>* times) {
  std::vector<long> out, out2;
  out.push_back(_label[lid].v);
  while( _parent.find(lid) != _parent.end() ) {
    out.push_back(_label[_parent[lid]].v);
    lid = _parent[lid];
  }
  path->clear();
  times->clear();
  for (size_t i = 0; i < out.size(); i++) {
    path->push_back(out[out.size()-1-i]);
    times->push_back(i); // unit time action.
  }
  return true;
};


void BOAst::AddNodeCstr(long nid, long t) {
  if ( _avl_node.find(nid) == _avl_node.end() ) {
    _avl_node[nid] = basic::AVLTree<long>();
  }
  _avl_node[nid].Add(t); // add this unsafe interval.
  if (t > _last_nc_t) {
    _last_nc_t = t;
  }
  return;
};

void BOAst::AddEdgeCstr(long u, long v, long t) {
  if ( _avl_edge.find(u) == _avl_edge.end() ) {
    _avl_edge[u] = std::unordered_map<long, basic::AVLTree<long> >();
  }
  if ( _avl_edge[u].find(v) == _avl_edge[u].end() ) {
    _avl_edge[u][v] = basic::AVLTree<long>();
  }
  _avl_edge[u][v].Add(t);
  return;
};

bool BOAst::_CollideCheck(long v1, long v2, long t) {
  // return true if in collision; return false if not in collision
  // node constraint check
  if (_avl_node[v2].Find(t+1).h != 0) {
    // the input t overlaps with exact a node constraint.
    return true;
  }
  // edge constraint check
  if (_avl_edge.find(v1) == _avl_edge.end() ) {
    return false;
  }
  if (_avl_edge[v1].find(v2) == _avl_edge[v1].end() ) {
    return false;
  }
  if (_avl_edge[v1][v2].Find(t).h == 0 ) {
    return false;
  }
  return true;
};

bool BOAst::_FrontierCheck(LabelST& l) {
  auto str = _L2S(l);
  if (_alpha.find(str) == _alpha.end()) {return false;}
  return _alpha[str].Check(l.g);
};

bool BOAst::_SolutionCheck(LabelST& l) {
  if (_sols.label_ids.size() == 0) {return false;}
  return _sols.Check(l.f);
};

void BOAst::_UpdateFrontier(LabelST& l) {
  auto str = _L2S(l);
  if (_alpha.find(str) == _alpha.end()) {
    if (DEBUG_BOA > 2) {
      std::cout << "[DEBUG] new frontier-naive for label " << l << std::endl;
    }
    _alpha[str] = FrontierBOA();
  }
  _alpha[str].Update(l);
  return ;
};

void BOAst::_UpdateSol(LabelST& l) {
  _sols.Update(l);
  return;
};

bool BOAst::_ReachGoalCondition(LabelST& l) {
  if ( (l.v == _vd) && (l.t > _last_nc_t) ) {
    return true;
  }
  return false;
};

basic::CostVector BOAst::_GetWaitCost(long v1, long v2, long dt) {
  // dt not in use.
  // v2 is the arrival node of the edge (v1,v2)
  if (_wait_cvec.size() > 0) {
    return _wait_cvec;
  }else{
    // no _wait_cvec available. i.e. _wait_cvec has not been set yet.
    return _graph->GetCost(v1,v2);
  }
};



/////////////////////////////////////////////////////////////////
////////////////// RunBOAst /////////////////////
/////////////////////////////////////////////////////////////////

int RunBOAstGrid(
  basic::GridkConn& g, long vo, long vd, double time_limit, basic::CostVector& wait_cost,
  std::vector< std::vector<long> >& ncs, std::vector< std::vector<long> >& ecs, MOSPPResult* res) 
{
  if (g.GetCostDim() != 2) {
    std::cout << " g.GetCostDim = " << g.GetCostDim() << std::endl;
    throw std::runtime_error("[ERROR] What?? are you invoking BOA* with more than two objective !?");
  }

  BOAst planner;
  planner.SetGrid(g); // polymorphism
  // std::cout << " BOAst search " << std::endl;
  planner.SetWaitCost(wait_cost);
  for (auto nc: ncs) {
    planner.AddNodeCstr(nc[0], nc[1]);
  }
  for (auto ec: ecs) {
    planner.AddEdgeCstr(ec[0], ec[1], ec[2]);
  }
  int outFlag = planner.Search(vo, vd, time_limit);
  *res = planner.GetResult();
  return outFlag;
};


} // end namespace search
} // end namespace rzq
