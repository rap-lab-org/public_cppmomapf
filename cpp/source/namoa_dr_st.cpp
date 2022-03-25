
/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/

#include "namoa_dr_st.hpp"

#include <set>
#include <memory>
#include <chrono>

namespace rzq{
namespace search{

basic::CostVector _proj(basic::CostVector v) {
  basic::CostVector out;
  for (size_t i = 1; i < v.size(); i++){
    out.push_back(v[i]);
  }
  return out;
};


FrontierLinear::FrontierLinear() {
  return;
};

FrontierLinear::~FrontierLinear() {
  return;
};

bool FrontierLinear::Check(basic::CostVector g) {
  for (auto iter : labels) {
    if (EpsDominance(iter.second.g, g)) {return true;}
  }
  return false;
};

bool FrontierLinear::drCheck(basic::CostVector g) {
  for (auto iter : labels) {
    if ( EpsDominance(_proj(iter.second.g),_proj(g)) ) {return true;}
  }
  return false;
};

void FrontierLinear::Update(LabelST& l, std::unordered_set<long> *deleted) {
  Filter(l, deleted);
  Add(l);
  return;
};

bool FrontierLinear::Remove(LabelST& l) {

  for (auto iter: labels) {
    if (l.g==iter.second.g) {
      labels.erase(iter.first);
      return true;
    }
  }
  return false;
};

void FrontierLinear::Add(LabelST& l) {
  this->labels[l.id] = l;
  return;
};


void FrontierLinear::Filter(LabelST& l, std::unordered_set<long> *deleted) {
  auto newLabels = labels;
  for (auto iter : labels) {
    if (EpsDominance(l.g, iter.second.g)) {
      newLabels.erase(iter.first);
      if (deleted) {
        deleted->insert(iter.first);
      }
    }
  }
  labels = newLabels;
  return;
};


////////////////////

NAMOAdrst::NAMOAdrst() {};

NAMOAdrst::~NAMOAdrst() {};

int NAMOAdrst::Search(long vo, long vd, double time_limit) {
  // ### init heu ###
  InitHeu(vd);

  // ### init ###
  auto tstart = std::chrono::steady_clock::now();
  _vo = vo;
  _vd = vd;
  basic::CostVector zero_vec;
  zero_vec.resize(_graph->GetCostDim(), 0);
  LabelST lo(_GenLabelId(), vo, zero_vec, _Heuristic(_vo), 0);
  _label[lo.id] = lo;
  _res.n_generated++;
  _open.insert( std::make_pair(lo.f, lo.id) );

  if (DEBUG_NAMOADRST > 0) {
    std::cout << "[DEBUG] Init, lo = " << lo << std::endl;
  }

  _res.success = true;

  // ### main search loop ###
  while ( !_open.empty() ) {

    // ## select label l, lexicographic order ##
    LabelST l = _label[ _open.begin()->second ];
    _open.erase(_open.begin());

    _Gop2Gcl(l);

    if (DEBUG_NAMOADRST > 0) {
      std::cout << "[DEBUG] ### Pop l = " << l << std::endl;
    }

    if ( _ReachGoalCondition(l) ) {
      // reach goal.
      _AddSols(l);
      _FilterOpenF(l);
      continue;
    }

    if (DEBUG_NAMOADRST > 1) {
      std::cout << "[DEBUG] ### Exp. " << l << std::endl;
    }

    auto tnow = std::chrono::steady_clock::now();
    auto time_passed = std::chrono::duration<double>(tnow-tstart).count();
    if (time_passed > time_limit) {
      if (DEBUG_NAMOADRST > 1) {
        std::cout << "[DEBUG] ### NAMOA_ST - TIMEOUT !! " << l << std::endl;
      }
      _res.success = false;
      break;
    }

    // ## expand label l ##
    _Expand(l);
  };

  // ### post-process the results ###

  if (_sol_label_ids.size() > 0) {
    for (auto lid : _sol_label_ids) {
      _res.paths[lid] = std::vector<long>();
      _res.times[lid] = std::vector<long>();
      _BuildPath(lid, &(_res.paths[lid]), &(_res.times[lid]));
      _res.costs[lid] = _label[lid].g;
    }
  }
  auto tend = std::chrono::steady_clock::now();
  _res.rt_search = std::chrono::duration<double>(tend-tstart).count();
  return 1;
};


void NAMOAdrst::_Gop2Gcl(LabelST& l) {

  auto str = _L2S(l);
  _g_op[str].Remove(l);
  if (_g_cl.find(str) == _g_cl.end()) {
    _g_cl[str] = FrontierLinear();
  }
  _g_cl[str].Add(l);
  return;
};

void NAMOAdrst::_AddSols(LabelST& l) {
  _sol_label_ids.push_back(l.id);
  return ;
};

void NAMOAdrst::_FilterOpenF(LabelST& l) {
  for (auto iter = _open.begin(); iter != _open.end(); ) {
    if (EpsDominance(l.g, iter->first)) { // g(l)=f(l) vs f(l')
      iter = _open.erase(iter);
    }else{
      ++iter;
    }
  }
  return ;
};

void NAMOAdrst::_Expand(LabelST& l) {
  
  _res.n_expanded++;
  std::unordered_set<long> succs = _graph->GetSuccs(l.v);
  succs.insert(l.v); // stay in place.

  for (const auto& u : succs) { // loop over vertices
    if (DEBUG_NAMOADRST > 0) {
      std::cout << "[DEBUG] >>>> Loop auto u : succs, u = " << u << std::endl;
    }

    // move from (v,t) to (u,t+1), collision check
    if ( _CollideCheck(l.v, u, l.t) ) {
      continue; // this ngh (u,t) is in conflict, skip.
    }

    // compute new cost vector and generate label.
    basic::CostVector new_g = l.g + _GetEdgeCost(l.v, u);
    if (u == l.v) { // wait in place action.
      new_g = l.g + _GetWaitCost(l.v, u, 1);
    }
    
    LabelST l2(_GenLabelId(), u, new_g, new_g+_Heuristic(u), l.t+1); // generate label.
    _label[l2.id] = l2;
    _parent[l2.id] = l.id;

    if ( _GopGclDomCheck(l2) ) {
      if (DEBUG_NAMOADRST > 0) {
        std::cout << "[DEBUG] >>>> --- GopGclDomCheck filtered" << std::endl;
      }
      continue;
    }

    l2.f = l2.g + _Heuristic(l2.v);

    if ( _SolDomCheck(l2.f) ) {
      if (DEBUG_NAMOADRST > 0) {
        std::cout << "[DEBUG] >>>> --- SolDomCheck filtered" << std::endl;
      }
      continue;
    }

    if (DEBUG_NAMOADRST > 0) {
      std::cout << "[DEBUG] >>>> Loop v= " << u << " gen and add to open, l' = " << l2 << std::endl;
    }

    _FilterGopAndOpenAndAdd(l2);
    _FilterGcl(l2);

  } // end for u

  return ;
};

bool NAMOAdrst::_GopGclDomCheck(LabelST& l) {
  std::string str = _L2S(l);
  if ( _g_op.find(str) != _g_op.end() ) {
    if (_g_op[str].Check(l.g)){
      return true; // should discard
    }
  }
  if ( _g_cl.find(str) != _g_cl.end() ) {
    if (_g_cl[str].drCheck(l.g)){ // NAMOA*-dr only do dim-reduction to Gcl.
      return true; // should discard
    }
  }
  return false;
};

void NAMOAdrst::_FilterGopAndOpenAndAdd(LabelST& l) {
  std::string str = _L2S(l);

  // filter Gop and Add to Gop.
  std::unordered_set<long> deleted_label_ids;
  if ( _g_op.find(str) != _g_op.end() ) {
    _g_op[str].Update(l, &deleted_label_ids);
  } else {
    _g_op[str] = FrontierLinear();
    _g_op[str].Add(l);
  }

  // filter OPEN as NAMOA* requires, can be expensive
  if (deleted_label_ids.size() > 0) {
    int counter = 0;
    for (auto iter = _open.begin(); iter != _open.end(); ) {
      if (deleted_label_ids.find(iter->second) != deleted_label_ids.end()) {
        iter = _open.erase(iter);
        counter++;
      }else{
        ++iter;
      }
      if (counter == deleted_label_ids.size()) {break;}
    }
  }

  // add to OPEN
  _res.n_generated++;
  _open.insert( std::make_pair(l.f, l.id) );

  return;
};


void NAMOAdrst::_FilterGcl(LabelST& l) {
  std::string str = _L2S(l);
  // filter Gcl.
  if ( _g_cl.find(str) != _g_cl.end() ) {
    _g_cl[str].Filter(l);
  }
  return ;
};


bool NAMOAdrst::_SolDomCheck(basic::CostVector& f) {
  for (auto iter : _sol_label_ids) {
    if ( EpsDominance(_proj(_label[iter].g), _proj(f)) ) { // dimensionality reduction
      return true;
    }
  }
  return false;
};


/////////////////////////////////////////////////////////////////
////////////////// RunNAMOA*dr-st /////////////////////
/////////////////////////////////////////////////////////////////

int RunNAMOAdrstGrid(
  basic::GridkConn& g, long vo, long vd, double time_limit, basic::CostVector& wait_cost,
  std::vector< std::vector<long> >& ncs, std::vector< std::vector<long> >& ecs, MOSPPResult* res) 
{
  NAMOAdrst planner;
  planner.SetGrid(g);
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
