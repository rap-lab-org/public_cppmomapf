
/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/

#include "momstar.hpp"
#include "union_find.hpp"

namespace rzq{
namespace mapf{

std::ostream& operator<<(std::ostream& os, MState& s)
{
  os << "{id:" << jv2str(s.jv) << ",g:" << (s.g) << "}";
  return os;
};

bool IsColSetSubset(const std::unordered_map<int,int>& a, const std::unordered_map<int,int>& b) 
{
  for (auto iter = a.begin(); iter != a.end(); iter++) {
    if (b.find(iter->first) == b.end()) { // element *iter in a but not in b.
      return false; 
    }
  }
  return true;
};

/**
 * @brief take the combination by select an element from every 
 *  sub-vector in the input vec.
 */
template <typename DataType>
void TakeCombination(
  const std::vector< std::vector<DataType> >& ivec, std::vector< std::vector<DataType> >* out_ptr) 
{
  std::vector< std::vector<DataType> >& out = *out_ptr;
  auto l = ivec.size();

  std::vector<int> indices;
  indices.resize(l,0);

  std::vector<DataType> a;
  a.resize(l);
  while (true) {
    // generate new element and insert into output result.
    for (int ri = 0; ri < l; ri++) {
      a[ri] = ivec[ri][indices[ri]];
    }
    out.push_back(a);
    // compute next indices
    indices[0]++;
    for (int ri = 0; ri < l-1; ri++) {
      if (indices[ri] == ivec[ri].size()) {
        indices[ri] = 0;
        indices[ri+1]++;
      }else{ break; }
    }
    if (indices[l-1] == ivec[l-1].size()) {
      break;
    }
  }
  return ;
};

////////////////////////////////////////////////////////////////////
////////////////////// MOMStarPolicy ///////////////////////
////////////////////////////////////////////////////////////////////

MOMStarPolicy::MOMStarPolicy() {};

MOMStarPolicy::~MOMStarPolicy() {};

bool MOMStarPolicy::Compute(long vd, double time_limit) {
  // exhaustive search
  int good = this->Search(vd, -1, time_limit) ;
  if (good == 0) {
    return false;
  }
  // build policy
  for (auto& iter : _alpha) {
    long src = iter.first;
    for (auto& lid : iter.second->label_ids) {
      if (_parent[lid] == -1) {
        continue;
      }
      long tgt = _label[_parent[lid]].v;
      if (tgt == src) {
        continue;
      }
      // add to phi, update _h
      if (_phi.find(src) == _phi.end()){
        _phi[src] = std::unordered_set<long>();
      }
      _phi[src].insert(tgt);
      if (_h.find(src) == _h.end()) {
        _h[src] = _label[lid].g ;
      }else{
        _h[src] = _h[src].ElemWiseMin( _label[lid].g ) ;
      }
    }
  }
  // handle edge case, if reach goal, stay at goal
  _phi[vd] = std::unordered_set<long>();
  _phi[vd].insert(vd);
  _h[vd] = basic::CostVector(0,_graph->GetCostDim());
  return true;
};


void MOMStarPolicy::Print() {
  std::cout << "---policy---" << std::endl;
  for (auto iter : _phi) {
    std::cout << iter.first << ":";
    for (auto jter : iter.second) {
      std::cout << jter << ",";
    }
    std::cout << std::endl;
  }
  std::cout << "---heu---" << std::endl;
  for (auto iter : _h) {
    std::cout << iter.first << ":" << iter.second << std::endl;
  }
};

std::vector<long>  MOMStarPolicy::Phi(const long& v) {
  if (_phi.find(v) == _phi.end()) {
    std::cout << "[ERROR] MOMStarPolicy::Phi, unknown input v = " << v << std::endl;
    Print();
    throw std::runtime_error( "[ERROR] MOMStarPolicy::Phi, unknown input v!" );
  }
  std::vector<long> out;
  for (auto u : _phi[v]) {
    out.push_back(u);
  }
  return out;
};

basic::CostVector MOMStarPolicy::H(const long& v) {
  return _h[v];
};

void MOMStarPolicy::InitHeu(long vd) {
  return;
};

basic::CostVector MOMStarPolicy::_Heuristic(long v) {
  auto out = basic::CostVector(0, _graph->GetCostDim());
  return out;
};

bool MOMStarPolicy::_SolutionCheck(search::LabelMOA l) {
  return false;
};

basic::CostVector MOMStarPolicy::_GetEdgeCost(const long& v1, const long& v2) {
  return _graph->GetCost(v2,v1) ; // search backwards
};

////////////////////////////////////////////////////////////////////
////////////////////// MOMStar ///////////////////////
////////////////////////////////////////////////////////////////////

MOMStar::MOMStar() {};

MOMStar::~MOMStar() {};

void MOMStar::SetGrid(size_t n_agents, basic::GridkConn& g, std::vector< std::vector<long> >& wait_costs) {
  _nAgent = n_agents;
  _g = &g;
  for (int i = 0; i < _nAgent; i++){
    _agentCosts[i] = basic::CostVector(wait_costs[i]);
  }
  return ;
};

void MOMStar::Search(
  std::vector<long>& starts, std::vector<long>& goals, double time_limit, double wH) 
{
  // init search
  _vo = starts;
  _vd = goals;
  _tlimit = time_limit;
  _wH = wH;
  _Init();
  if ( DEBUG_MOMSTAR ) { std::cout << "[DEBUG] after init..." << std::endl; }

  // prepare for search
  _result.find_all_pareto = false;

  // main while loop
  while ( true ) {
    // check termination
    if ( _open.empty() ) {
      if (_sol.size() > 0) {
        std::cout << "[DEBUG] MOMStar::Search, find all pareto !" << std::endl;
        _result.find_all_pareto = true; 
      }
      break;
    }
    // check timeout
    if ( std::chrono::duration<double>(std::chrono::steady_clock::now() - _t0).count() > _tlimit ) {
      std::cout << "[INFO] MOMStar::Search times out!" << std::endl;
      break; // time out!
    }

    // pop from open
    MState& s = _states[ _open.begin()->second ];
    _open.erase(_open.begin());
    if ( DEBUG_MOMSTAR ) { std::cout << "[DEBUG] #### pop from open state : " << s << std::endl; }

    // check for solution
    if (_IfReachGoal(s.jv)) {
      _sol.push_back(s.id); // new solution found.
      if (_sol.size() == 1) {
        // first sol
        _result.rt_firstSol = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - _t0).count();
      }
      continue;
    }

    // expand
    std::vector< std::vector<long> > nghs;
    bool success = _GetLimitNgh(s.id, &nghs);
    if (!success) {
      break;
    }
    _result.n_expanded++;

    for (auto& ngh : nghs) { // loop over all limited neighbors.

      auto colSet = _ColCheck(s.jv, ngh);
      if (colSet.size() > 0) { // there is collision.
        _BackProp(s.id, colSet);
        continue;
      }

      // collision-free
      // get cost vector
      auto cuv = _GetTransCost(s.jv, ngh, s.id);
      basic::CostVector g_ngh = s.g + cuv;
      basic::CostVector f_ngh = g_ngh + _H(ngh); // note that _wH is considered within _H()
      if ( _SolFilter(f_ngh) ) { // dominated by some already found solution.
        continue;
      }

      // dom check
      std::vector<long> dom_ids;
      if ( _DomCompare(ngh, g_ngh, &dom_ids) ) {
        // do dom-back-prop.
        for (const auto& dom_id : dom_ids) {
          _BackProp(s.id, _states[dom_id].colSet);
          _AddBackSet(s.id, dom_id);
        }
      }else{
        // add to open.
        long curr_id = _GenId();
        _states[curr_id] = MState();
        _states[curr_id].id = curr_id;
        _states[curr_id].jv = ngh;
        _states[curr_id].g = g_ngh;
        _states[curr_id].colSet = std::unordered_map<int, int>(); // collision set, empty set.
        _open.insert( std::make_pair(f_ngh, curr_id) );
        _result.n_generated++;
        auto ngh_str = jv2str(ngh);
        if (_frontiers.find(ngh_str) == _frontiers.end()) {
          _frontiers[ngh_str] = std::unordered_set<long>();
        }
        _frontiers[ngh_str].insert(curr_id);
        _AddBackSet(s.id, curr_id);
        _parent[curr_id] = s.id; // track parent.
      }
    }
  } // end while loop

  _result.rt_search = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - _t0).count();
  _PostProcessResult();

  return ;
};

MOMAPFResult MOMStar::GetResult() const {
  return _result;
};

bool MOMStar::_GetLimitNgh(const long& sid, std::vector< std::vector<long> >* out) 
{
  out->clear();
  const auto& colSet = _states[sid].colSet;
  const auto& jv = _states[sid].jv;
  std::vector< std::vector<long> > ngh_vec;
  ngh_vec.resize(_nAgent);
  long ngh_size = 1;
  for (int ri = 0; ri < _nAgent; ri++){ // loop over robots
    if (colSet.find(ri) == colSet.end()){ // not in collision
      ngh_size *= _policies[ri].Phi(jv[ri]).size();
      for (const auto& u : _policies[ri].Phi(jv[ri]) ){
        ngh_vec[ri].push_back(u);
      }
    }else{ // in collision
      auto idvl_nghs = _g->GetSuccs(jv[ri]);
      ngh_size *= idvl_nghs.size();
      for (const auto& u : idvl_nghs) {
        ngh_vec[ri].push_back(u);
      }
      ngh_vec[ri].push_back(jv[ri]); // wait in place
    }
  }

  _result.n_branch += ngh_size;
  if (ngh_size > MAX_NGH_SIZE) {
    return false;
  }

  TakeCombination(ngh_vec, out);
  return true;
};


bool MOMStar::_Init() {

  if ( _nAgent != _vo.size() ){
    throw std::runtime_error("[ERROR] MOMStar::_Init, _nAgent != _vo.size() !") ;
  }

  // set up timer
  _t0 = std::chrono::steady_clock::now(); // for timing.
  _result.n_generated = 0;
  _result.n_expanded = 0;
  _result.rt_initHeu = 0.0;
  _result.n_maxColSet = 0;
  _result.n_branch = 0;

  // init policies.
  for (int ri = 0; ri < _nAgent; ri++) {
    if ( DEBUG_MOMSTAR ) { std::cout << "[DEBUG] init, construct policy for agent " << ri << std::endl; }
    _policies[ri] = MOMStarPolicy();
    _policies[ri].SetGraphPtr(_g);
    double remain_time = _tlimit - std::chrono::duration<double>(
      std::chrono::steady_clock::now() - _t0).count();
    bool good = _policies[ri].Compute(_vd[ri], remain_time);
    if (!good) { // timeout for policy computation.
      return false;
    }
  }
  _result.rt_initHeu = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - _t0).count();

  // init search
  long curr_id = _GenId();
  _parent[curr_id] = -1;
  _states[curr_id] = MState();
  _states[curr_id].id = curr_id;
  _states[curr_id].jv = _vo;
  _states[curr_id].g = basic::CostVector(0, _g->GetCostDim());
  _states[curr_id].colSet = std::unordered_map<int, int>(); // collision set, empty set.
  _open.insert( std::make_pair(_H(_vo), curr_id) );
  _result.n_generated++;
  _frontiers[jv2str(_vo)] = std::unordered_set<long>();
  _frontiers[jv2str(_vo)].insert(curr_id);
  return true;
};

basic::CostVector MOMStar::_H(const std::vector<long>& jv) {
  basic::CostVector out(0, _g->GetCostDim());
  for (int ri = 0; ri < _nAgent; ri++) {
    out += _policies[ri].H(jv[ri]);
  }
  if (_wH > 1.0) { // heuristic inflation
    for (size_t j = 0; j < out.size(); j++) {
      out[j] = long(out[j] * _wH) ; // NOTE the long conversion here !
    }
  }
  return out;
};

bool MOMStar::_IfReachGoal(const std::vector<long>& jv) {
  return jvEqual(jv, _vd);
};

std::unordered_map<int,int> MOMStar::_ColCheck(
  const std::vector<long>& jv1, const std::vector<long>& jv2) 
{
  std::unordered_map<int,int> out;
  for (int ix = 0; ix < _nAgent; ix++) { // loop over every pair of agents
    for (int iy = ix+1; iy < _nAgent; iy++) {
      if ( (jv2[ix] == jv2[iy]) || ((jv1[ix] == jv2[iy]) && (jv1[iy] == jv2[ix])) )
      { // vertex conflict or swap locations conflict
        if (out.find(ix) == out.end()) { // ix is new
          if (out.find(iy) == out.end()) { // iy is new
            out[ix] = iy;
            out[iy] = iy;
          }else{ // iy is already there
            out[ix] = out[iy];
          }
        } // end of if
        else { // ix is not new
          if (out.find(iy) == out.end()) { // iy is new
            out[iy] = out[ix];
          }else{ // both are not new
            rzq::basic::UFUnion(&out, ix, iy);
          }
        } // end of else
      } // end of if
    } // end for iy
  } // end for ix
  return out;
};


void MOMStar::_AddBackSet(const long& sid1, const long& sid2) 
{
  if (back_set_map_.find(sid2) == back_set_map_.end()) {
    back_set_map_[sid2] = std::unordered_set<long>();
  }
  back_set_map_[sid2].insert(sid1);
  return;
};

void MOMStar::_BackProp(const long& sid, const std::unordered_map<int,int>& col_set) {
  if ( IsColSetSubset(col_set, _states[sid].colSet) ) {
    return;
  }else{
    _ColSetUnion(col_set, &(_states[sid].colSet) );
    if (_states[sid].colSet.size() > _result.n_maxColSet) {
      _result.n_maxColSet = _states[sid].colSet.size();
    }
    _Reopen(sid);
    if (back_set_map_.find(sid) == back_set_map_.end()) {
      return;
    }
    auto& bset = back_set_map_[sid];
    for (auto iter = bset.begin(); iter != bset.end(); iter++) {
      _BackProp(*iter, col_set);
    }
  }
};

void MOMStar::_ColSetUnion(const std::unordered_map<int,int>& a, std::unordered_map<int,int>* b) 
{
  for (auto iter = a.begin(); iter != a.end(); iter++) {
    if (b->find(iter->first) == b->end()) { // element *iter in a but not in b.
      (*b)[iter->first] = iter->second;
    }
  }
  return ;
}

void MOMStar::_Reopen(const long& sid) {
  auto& s = _states[sid];
  if (DEBUG_MOMSTAR) {std::cout << "[DEBUG] reopen : " << s << std::endl; }
  _open.insert( std::make_pair( s.g+_H(s.jv), sid) );
  return;
}; 

basic::CostVector MOMStar::_GetTransCost(
  const std::vector<long>& jv1, const std::vector<long>& jv2, long curr_id) 
{
  basic::CostVector out(0, _g->GetCostDim());
  for (size_t ix = 0; ix < _nAgent; ix++) { // loop over all agents
    if (jv1[ix] == _vd[ix] && jv2[ix] == jv1[ix]) { // stay at goal
      continue;
    } 
    if (jv2[ix] == jv1[ix]) {
      out += _agentCosts[ix]; 
      continue; 
    }
    if (jv1[ix] == _vd[ix] && jv2[ix] != _vd[ix]) { // move off goals
      out += _MoveFromGoalCost(ix, curr_id); // add accumulated wait cost at goal, because the robot moves away.
    }
    out += _g->GetCost(jv1[ix],jv2[ix]);
  }
  return out;
};

bool MOMStar::_SolFilter(const basic::CostVector& fu) {
  for (const auto& gid : _sol) {
    if ( search::EpsDominance(_states[gid].g, fu) ) {
      _result.n_solFiltered++;
      return true;
    }
  }
  return false;
};

bool MOMStar::_DomCompare(
  const std::vector<long>& ju, const basic::CostVector& gu, std::vector<long>* dom_ids) 
{
  dom_ids->clear();
  for ( const auto& sid : _frontiers[jv2str(ju)] ) {
    if ( search::EpsDominance(_states[sid].g, gu) ) {
      dom_ids->push_back(sid);
    }
  }
  if (dom_ids->size() > 0) {
    return true;
  }
  return false;
};


void MOMStar::_PostProcessResult() 
{
  for (const auto& gid : _sol) {
    std::vector< std::vector<long> > jp;
    _BuildJointPath(gid, &jp);
    _result.solus[gid] = PathSet();
    _JPath2PathSet(jp, &(_result.solus[gid]));
    _result.costs[gid] = _states[gid].g;
  }
  _result.n_initRoot = -1;

  _result.n_branch = 1.0*_result.n_branch / _result.n_expanded;
  return ;
};

void MOMStar::_BuildJointPath(long sid, std::vector< std::vector<long> >* jp) 
{
  std::vector< std::vector<long> > reversed_jp;
  while (sid != -1) {
    reversed_jp.push_back(_states[sid].jv);
    sid = _parent.at(sid);
  }
  jp->clear();
  for (int idx = reversed_jp.size()-1; idx >= 0; idx--) { // convert to right order.
    jp->push_back(reversed_jp[idx]);
  }
  return ;
};

void MOMStar::_JPath2PathSet(const std::vector< std::vector<long> >& jp, PathSet* ps)
{
  ps->clear();
  for (int ri = 0; ri < _nAgent; ri++) {
    (*ps)[ri] = std::vector<long>();
    (*ps)[ri].resize(jp.size());
    for (size_t j = 0; j < jp.size(); j++){
      (*ps)[ri][j] = jp[j][ri];
    }
  }
  return ;
};

basic::CostVector MOMStar::_MoveFromGoalCost(const int& ri, long sid) {
  basic::CostVector out(0,_g->GetCostDim());
  long v = _states[sid].jv[ri];
  while (true) {
    long pid = _parent[sid];
    long u = _states[pid].jv[ri];
    if ((u == v) && (u == _vd[ri])) {
      out += _agentCosts[ri];
    }else{
      break;
    }
    sid = pid;
  }
  return out;
};

////////////////////////////////////////////////////////////////////
////////////////////// Others ///////////////////////
////////////////////////////////////////////////////////////////////

void CompileHelper() {
  std::vector< std::vector<long> > a1;
  TakeCombination( std::vector< std::vector<long> >(), &a1 );
  std::vector< std::vector<int> > a2;
  TakeCombination( std::vector< std::vector<int> >(), &a2 );
  std::vector< std::vector<short> > a3;
  TakeCombination( std::vector< std::vector<short> >(), &a3 );
  std::vector< std::vector<float> > a4;
  TakeCombination( std::vector< std::vector<float> >(), &a4 );
  std::vector< std::vector<double> > a5;
  TakeCombination( std::vector< std::vector<double> >(), &a5 );
  std::vector< std::vector<std::string> > a6;
  TakeCombination( std::vector< std::vector<std::string> >(), &a6 );
  std::vector< std::vector<bool> > a7;
  TakeCombination( std::vector< std::vector<bool> >(), &a7 );
}

} // end namespace mapf
} // end namespace rzq
