
/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/

#include "moa.hpp"

#include <set>
#include <memory>
#include <chrono>

namespace rzq{
namespace search{

std::ostream& operator<<(std::ostream& os, LabelMOA& l)
{
  std::string s;
  s = "{id:" + std::to_string(l.id) + ",v:" + std::to_string(l.v) + ",g:" 
    + l.g.ToStr() + ",f:" + l.f.ToStr() + "}";
  os << s;
  return os;
};

MOA::MOA() {};

MOA::~MOA() {
  for (auto k :_alpha) { // free all frontier that are newed.
    delete k.second;
  }
};

void MOA::SetGraphPtr(basic::Graph* g) {
  _graph = g;
};

void MOA::InitHeu(long vd) {
  auto tstart = std::chrono::steady_clock::now();
  _dijks.resize(_graph->GetCostDim());
  for (size_t i = 0; i<_graph->GetCostDim(); i++) {
    _dijks[i].SetGraphPtr(_graph);
    _dijks[i].Search(vd, i);
  }
  auto tend = std::chrono::steady_clock::now();
  _res.rt_search = std::chrono::duration<double>(tend-tstart).count();
  return ;
};

int MOA::Search(long vo, long vd, double time_limit) {
  // ### init heu ###
  InitHeu(vd);

  // ### init ###
  auto tstart = std::chrono::steady_clock::now();
  _vo = vo;
  _vd = vd;
  basic::CostVector zero_vec;
  zero_vec.resize(_graph->GetCostDim(), 0);
  LabelMOA lo(_GenLabelId(), vo, zero_vec, _Heuristic(_vo));
  _label[lo.id] = lo;
  _res.n_generated++;
  // _UpdateFrontier(lo);
  _open.insert( std::make_pair(lo.f, lo.id) );

  if (DEBUG_MOA > 0) {
    std::cout << "[DEBUG] Init, lo = " << lo << std::endl;
  }

  // ### main search loop ###
  while ( !_open.empty() ) {
    // ## select label l, lexicographic order ##
    LabelMOA l = _label[ _open.begin()->second ];
    _open.erase(_open.begin());

    if (DEBUG_MOA > 0) {
      std::cout << "[DEBUG] ### Pop l = " << l << std::endl;
    }

    auto tnow = std::chrono::steady_clock::now();
    if ( std::chrono::duration<double>(tnow-tstart).count() > time_limit) {
      return 0; // fails. timeout.
    }

    // ## lazy dominance check ##
    if ( _FrontierCheck(l) || _SolutionCheck(l) ) {
      if (DEBUG_MOA > 1) {
        std::cout << "[DEBUG] F- AND S-check, dom, cont..." << std::endl;
      }
      continue;
    }
    if (l.v == vd) {
      // do nothing, frontier at vd is the same as solution set.
    }
    _UpdateFrontier(l);
    if (DEBUG_MOA > 1) {
      std::cout << "[DEBUG] ### Exp. " << l << std::endl;
    }

    // ## expand label l ##
    _res.n_expanded++;
    auto succs = _graph->GetSuccs(l.v);
    for (auto u : succs) {
      basic::CostVector gu = l.g + _GetEdgeCost(l.v, u);
      LabelMOA l2(_GenLabelId(), u, gu, gu + _Heuristic(u));
      _label[l2.id] = l2;
      _parent[l2.id] = l.id;
      if (DEBUG_MOA > 0) {
        std::cout << "[DEBUG] >>>> Loop v= " << u << " gen l' = " << l2 << std::endl;
      }
      if (_FrontierCheck(l2)) {
        if (DEBUG_MOA > 1) {
          std::cout << "[DEBUG] ----- F-Check, dom, cont..." << std::endl;
        }
        continue;
      }
      if (DEBUG_MOA > 0) {
        std::cout << "[DEBUG] ----- Add to open..." << std::endl;
      }
      _res.n_generated++;
      _open.insert( std::make_pair(l2.f, l2.id) );
    }
  };

  // ### post-process the results ###
  if (_alpha.find(vd) != _alpha.end()) {
    for (auto lid : _alpha[vd]->label_ids) {
      _res.paths[lid] = std::vector<long>();
      _res.times[lid] = std::vector<long>();
      bool ret_flag = _BuildPath(lid, &(_res.paths[lid]), &(_res.times[lid]) );
        // when this flag is used, remember to check here for safety.
      _res.costs[lid] = _label[lid].g;

      // _res.paths[lid] = _BuildPath(lid);
      // _res.costs[lid] = _label[lid].g;
    }
  }
  auto tend = std::chrono::steady_clock::now();
  _res.rt_search = std::chrono::duration<double>(tend-tstart).count();
  return 1;
};

basic::CostVector MOA::_Heuristic(long v) {
  auto out = basic::CostVector(0, _graph->GetCostDim());
  for (size_t cdim = 0; cdim < out.size(); cdim++) {
    out[cdim] = _dijks[cdim].GetCost(v);
    // out[cdim] = 0;
    if (out[cdim] < 0) {
      throw std::runtime_error( "[ERROR], unavailable heuristic !?" );
    }
  }
  // std::cout << " h(" << v << ") = " << out << std::endl;
  return out;
};

MOSPPResult MOA::GetResult() const {
  return _res;
};

long MOA::_GenLabelId() {
  return _label_id_gen++;
};

bool MOA::_FrontierCheck(LabelMOA l) {
  if (_alpha.find(l.v) == _alpha.end()) {return false;}
  return _alpha[l.v]->Check(l.g);
};

bool MOA::_SolutionCheck(LabelMOA l) {
  if (_alpha.find(_vd) == _alpha.end()) {return false;}
  return _alpha[_vd]->Check(l.f);
};

basic::CostVector MOA::_GetEdgeCost(const long& v1, const long& v2) {
  return _graph->GetCost(v1,v2) ; // search forwards
};

// std::vector<long> MOA::_BuildPath(long lid) {
//   std::vector<long> out, out2;
//   out.push_back(_label[lid].v);
//   while( _parent.find(lid) != _parent.end() ) {
//     out.push_back(_label[_parent[lid]].v);
//     lid = _parent[lid];
//   }
//   for (size_t i = 0; i < out.size(); i++) {
//     out2.push_back(out[out.size()-1-i]);
//   }
//   return out2;
// };

bool MOA::_BuildPath(long lid, std::vector<long>* path, std::vector<long>* times) {
  std::vector<long> out, out2;
  out.push_back(_label[lid].v);
  // out2.push_back(_label[lid].t);
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


void MOA::SetGrid(basic::GridkConn& g) {
  temp_g = g;
  SetGraphPtr(&temp_g);
};

void MOA::_UpdateFrontier(LabelMOA l) {
  if (_alpha.find(l.v) == _alpha.end()) {
    if (DEBUG_MOA > 2) {
      std::cout << "[DEBUG] new frontier-naive for label " << l << std::endl;
    }
    _alpha[l.v] = new FrontierNaive;
  }
  _alpha[l.v]->Update(l);

  // debug info below
  if (DEBUG_MOA > 0) {
    std::cout << "[DEBUG] ----->> UpdateF. tree(" << l.v << ") : " << std::endl;
    std::cout << (*_alpha[l.v]) << std::endl;
  }
  if (DEBUG_MOA > 1) {
    std::cout << "[DEBUG] ----->> UpdateF. label ids = {";
    for (auto i : _alpha[l.v]->label_ids) {
      std::cout << i << ",";
    }
    std::cout << "} " << std::endl;
  }
  return ;
};

FrontierNaive::FrontierNaive() {
  return;
};

FrontierNaive::~FrontierNaive() {
  return;
};

bool FrontierNaive::Check(basic::CostVector g) {
  for (auto v: _nondom) {
    if (EpsDominance(v, g) ) {return true;}
  }
  return false;
};

void FrontierNaive::Update(LabelMOA l) {
  auto newND = _nondom;
  for (auto v: _nondom) {
    if (EpsDominance(l.g, v)) {
      newND.remove(v);
    }
  }
  newND.push_back(l.g);
  _nondom = newND;
  this->label_ids.insert(l.id);
  return ;
};

std::ostream& operator<<(std::ostream& os, FrontierNaive& f) {
  os << "{";
  for (auto v : f._nondom) {
    std::cout << v << ",";
  }
  os << "}";
  return os;
};

} // end namespace search
} // end namespace rzq
