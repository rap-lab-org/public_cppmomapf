/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/

#include "mocbs.hpp"

namespace rzq{
namespace mapf{


CBSConstraint::CBSConstraint() {};

CBSConstraint::CBSConstraint(int i0, int j0, long vi0, long vj0, long ta0, long tb0) {
  i = i0;
  j = j0;
  vi = vi0;
  vj = vj0;
  ta = ta0;
  tb = tb0;
  return;
};

std::ostream& operator<<(std::ostream& os, const CBSConstraint& c) {
  os << "CBSConflict{i:" << c.i << ",j:" << c.j << ",vi:" << c.vi << ",vj:" 
     << c.vj << ",ta:" << c.ta << ",tb:" << c.tb << "}";
  return os;
};

std::vector<CBSConstraint> CBSConflict::Split() {
  std::vector<CBSConstraint> out;
  out.push_back( CBSConstraint(i,j,vi,vj,ta,tb) );
  out.push_back( CBSConstraint(j,i,vj,vi,ta,tb) );
  return out;
};

std::ostream& operator<<(std::ostream& os, const CBSConflict& c) {
  os << "CBSConflict{i:" << c.i << ",j:" << c.j << ",vi:" << c.vi << ",vj:" 
     << c.vj << ",ta:" << c.ta << ",tb:" << c.tb << "}";
  return os;
};


CBSConflict DetectConflict(std::vector<long>* p1, std::vector<long>* p2) {
  CBSConflict cstr;
  if ( (p1->size() == 0) || (p2->size() == 0) ) {
    std::cout << "[WARNING] DetectConflict receive zero size path !" << std::endl;
    // throw std::runtime_error("[WARNING] DetectConflict receive zero size path !");
    return cstr;
  }
  size_t l = p1->size() > p2->size() ? p1->size() : p2->size(); // find max.
  for (size_t idx = 0; idx < l; idx++) {
    long v1 = idx < p1->size() ? p1->at(idx) : p1->back();
    long v2 = idx < p2->size() ? p2->at(idx) : p2->back();
    if ( v1 == v2 ) { // vertex conflict
      cstr.vi = v1;
      cstr.vj = v2;
      cstr.ta = idx;
      cstr.tb = idx;
      break; // stop at the first conflict detected.
    }
    size_t idy = idx + 1;
    if (idy >= l) {continue;}
    long v1b = idy < p1->size() ? p1->at(idy) : p1->back();
    long v2b = idy < p2->size() ? p2->at(idy) : p2->back();
    if ( (v1 == v2b) && (v1b == v2) ) { // edge conflict
      cstr.vi = v1;
      cstr.vj = v2;
      cstr.ta = idx;
      cstr.tb = idy;
      break; // stop at the first conflict detected.
    }
  }
  return cstr;
};

CBSConflict DetectConflict(PathSet* p) {
  // std::cout << " >>>>>>> " << (*p) << std::endl;
  std::vector<int> agents;
  for (auto& it : (*p)) { // normally, these IDs are not in order.
    agents.push_back(it.first);
    // std::cout << " add agent i = " << it.first << " into agents" << std::endl;
  }
  CBSConflict cft;
  for (size_t ii = 0; ii < agents.size(); ii++) {
    for (size_t jj = ii+1; jj < agents.size(); jj++) {
      // std::cout << " ii " << ii << " jj " << jj << std::endl;
      cft = DetectConflict( &(p->at(agents[ii])), &(p->at(agents[jj])) );
      if (cft.IsValid()) { // specify agents
        cft.i = agents[ii];
        cft.j = agents[jj];
        return cft;
      }
    }
  }
  return cft;
};

bool UnitTimePath(std::vector<long>& p0, std::vector<long> t0,
  std::vector<long>* p1, std::vector<long>* t1, long dt) 
{
  if (t0.size() == 0) { // input path is empty
    std::cout << "[ERROR] UnitTimePath input times of zero size!" << std::endl;
    throw std::runtime_error("[ERROR] UnitTimePath input times of zero size!");
    return false;
  }
  long ct = t0[0]; // the start time.

  p1->clear();
  t1->clear();
  for (size_t ii = 0; ii < t0.size()-1; ii++){
    p1->emplace_back(p0[ii]);
    t1->emplace_back(t0[ii]);
    if (t0[ii+1] - t0[ii] > 1.0001) {
      long ct = t0[ii];
      while (t0[ii+1]-ct > 1.0001) {
        p1->emplace_back(p0[ii]);
        t1->emplace_back(ct+1);
        ct++;
      }
    }
  }
  p1->emplace_back(p0.back());
  t1->emplace_back(t0.back());
  return true;
};

std::ostream& operator<<(std::ostream& os, const CBSNode& n) {
  os << "CBSNode{id:" << n.id << ",parent:" << n.parent << ",cstr:" << n.cstr << ",g:" << n.g << "," << n.paths << "}";
  // paths, costs,
  // << ",ta:" << c.ta << ",tb:" << c.tb << "}";
  return os;
};

/////////////////////////////////////////////////////////////////////////
////////////////////// MOCBS_TB /////////////////////
/////////////////////////////////////////////////////////////////////////

MOCBS_TB::MOCBS_TB() {
  std::cout << "[INFO] Constructor - MOCBS_TB" << std::endl;
};

MOCBS_TB::~MOCBS_TB() {};

void MOCBS_TB::SetGraphPtr(int ri, size_t n_agents, basic::Graph* g, std::vector<long>& wait_cost) {
  if (ri < -1) {
    std::cout << "[ERROR] MOCBS_TB::SetGraphPtr, input ri = " << ri << ", don't know what to do..." << std::endl;
    throw std::runtime_error("[ERROR] MOCBS_TB::SetGraphPtr, check the input ri !");
    return;
  }
  _cdim = g->GetCostDim();
  _nAgent = n_agents;
  if (ri == -1) {
    for (int ri = 0; ri < n_agents; ri++){
      _agentGraphs[ri] = g;
      _agentCosts[ri] = basic::CostVector(wait_cost);
    }
  }else{
    _agentGraphs[ri] = g;
    _agentCosts[ri] = basic::CostVector(wait_cost);
  }
  // _cdim = _agentGraphs[ri]->GetCostDim();
  // _nAgent = n_agents;
  return;
};

void MOCBS_TB::SetGrid(int ri, size_t n_agents, basic::GridkConn& g, std::vector<long>& wait_cost) {
  if (ri < -1) {
    std::cout << "[ERROR] MOCBS_TB::SetGrid, input ri = " << ri << ", don't know what to do..." << std::endl;
    throw std::runtime_error("[ERROR] MOCBS_TB::SetGrid, check the input ri !");
    return;
  }

  _cdim = g.GetCostDim();
  _nAgent = n_agents;
  
  if (ri == -1) {
    for (int ri = 0; ri < n_agents; ri++){
      _tempAgentGrids[ri] = g;
      _agentCosts[ri] = basic::CostVector(wait_cost);
    }
  }else{
    _tempAgentGrids[ri] = g;
    _agentCosts[ri] = basic::CostVector(wait_cost);
  }
  return;
};

void MOCBS_TB::SetWaitCosts(std::vector<basic::CostVector>& wait_costs) {
  for (int i=0; i < wait_costs.size(); i++){
    _agentCosts[i] = wait_costs[i];
  }
};

void MOCBS_TB::Search(std::vector<long>& starts, std::vector<long>& goals, double time_limit, double eps) {
  
  // assign graph pointer here.
  for (int ri = 0; ri < _nAgent; ri++){
    SetGraphPtr(ri, _nAgent, &(_tempAgentGrids[ri]), _agentCosts[ri]);
  }
  // set epsilon, by default 0.
  _epsilon = eps;

  // copy
  if (starts.size() != _nAgent) {
    std::cout << "[ERROR] MOCBS_TB::Search input starts.size() = " << starts.size() 
              << " does not match _nAgent = " << _nAgent << std::endl;
    throw std::runtime_error("[ERROR] MOCBS_TB::Search input starts size mismatch!");
  }
  if (goals.size() != _nAgent) {
    std::cout << "[ERROR] MOCBS_TB::Search input goals.size() = " << starts.size() 
              << " does not match _nAgent = " << _nAgent << std::endl;
    throw std::runtime_error("[ERROR] MOCBS_TB::Search input goals size mismatch!");
  }
  _vo = starts;
  _vd = goals;
  _tlimit = time_limit;
  _t0 = std::chrono::steady_clock::now(); // for timing.

  // init
  _Init();
  _result.find_all_pareto = false;

  // main search loop
  size_t iterCount__ = 0;
  while ( true ) {
    iterCount__++;
    if (DEBUG_MOCBS > 0) {
      std::cout << "[DEBUG]----------------------- search iter : " 
        << iterCount__ << " ------------------------" << std::endl;
    }

    //// select high-level node, lexicographic order
    CBSNode *node = _SelectNode();
    
    //// see if open depletes and there is no more roots.
    if (!node) {
      _result.find_all_pareto = true;
      break; // terminates
    }

    //// solution filtering
    if ( _SolutionFilter(node->g) ) {
      if (DEBUG_MOCBS) {
        std::cout << "[DEBUG] * MOCBS_TB::Search node " << node->id << " solution filtered" << std::endl;
      }
      continue; // filtered, skip this node.
    };

    //// detect conflict
    auto cft = DetectConflict(&(node->paths));

    //// new solution
    if (!cft.IsValid()) {
      // find a conflict-free solution. add it to the result.
      _UpdateSolution(node);
      continue; // end this iteration
    }

    if ( _IfTimeout() ) {
      std::cout << "[INFO] * MOCBS_TB::Search Timeout, reach limit " << _tlimit << std::endl;
      break;
    };

    //// split conflict (i.e. expand *node )
    auto cstrs = cft.Split();
    for (auto cstr: cstrs) { // loop over both constraints.
      _Lsearch(cstr.i, node, &cstr);
    }
    _result.n_expanded++; // # conflict resolved.
  } // end while loop

  auto tnow = std::chrono::steady_clock::now(); // for timing.
  _result.rt_search = std::chrono::duration<double>(tnow - _t0).count();  
  _result.n_branch = _result.n_branch / _result.n_expanded; // average branching factor.
  // std::cout << "[INFO] * MOCBS_TB::Search, Bingo ! reach the end !!, find all = " << int(_result.find_all_pareto) << std::endl;
  return ;
};

MOMAPFResult MOCBS_TB::GetResult() const {
  return _result;
};

bool MOCBS_TB::_GetAllCstr(int ri, CBSNode *in, std::vector<CBSConstraint*> *out) {
  out->clear();
  long cid = in->id;
  while (cid != -1) {
    CBSNode* n = &(_nodes[cid]);
    CBSConstraint* cstr_ptr = &(n->cstr);
    if (cstr_ptr->i == ri) { // a cstr imposed on agent-i.
      out->push_back( &(n->cstr) );
    }
    cid = n->parent;
  }
  return true;
};

bool MOCBS_TB::_Init() {

  if (DEBUG_MOCBS > 0) {
    std::cout << "[DEBUG] * MOCBS_TB::_Init start " << std::endl;
  }

  auto init_t0 = std::chrono::steady_clock::now(); // for timing.
  _result.n_expandLow = 0.0;

  //// plan individual pareto-optimal path for each agent.
  std::vector<CBSConstraint*> emptySetCstr;
  _result.n_initRoot = 1;
  std::cout << "[INFO] #roots = (";
  for (int ri = 0; ri < _vo.size(); ri++) {
    _initRes[ri] = search::MOSPPResult();
    int ret_flag = _LsearchPlan( ri, emptySetCstr, &(_initRes[ri]) );
    
    _result.n_initRoot *= _initRes[ri].paths.size();
    // std::cout << "_initRes[ri] = " << _initRes[ri] << std::endl;
    std::cout << " " << _initRes[ri].paths.size() << " ";

    if (ret_flag == 0) {
      // TODO, how to handle this exception
      std::cout << "[ERROR] * MOCBS_TB::_Init failed at robot " << ri << std::endl;
      throw std::runtime_error( "[ERROR] MOCBS_TB::_Init failed !" );
    }
    __InitPathSetUp(ri);
  }

  //// generate the first root node. output via a pointer to that root node.
  long nid = _GenNodeId();
  _nodes[nid].id = nid;
  CBSNode* out = &(_nodes[nid]); // set as output pointer
  _GetPathSetGetCurr(out);

  _open.insert( std::make_pair(out->g, out->id) );
  _result.n_generated++; // update

  if (DEBUG_MOCBS > 0) {
    std::cout << "[DEBUG] * MOCBS_TB::_Init, node = " << _nodes[nid] << std::endl;
  }

  _result.n_branch = 0.0;
  _result.rt_initHeu = std::chrono::duration<double>(std::chrono::steady_clock::now() - init_t0).count();  

  std::cout << "), total = " << _result.n_initRoot << " rt_init = " << _result.rt_initHeu << std::endl;
  // std::cout << " At the end of _Init(), n_generated = " << _result.n_generated << std::endl;
  return true;
};

CBSNode* MOCBS_TB::_SelectNode() {
  if ( !_open.empty() ) {
    long nid = _open.begin()->second;
    CBSNode* out = &( _nodes[nid] );
    _open.erase(_open.begin());
    if (DEBUG_MOCBS > 0) {
      std::cout << "[DEBUG] ++ MOCBS_TB::_SelectNode node = " << (*out) << std::endl;
    }
    return out; // a node is selected.
  }

  // now _open is empty, see if we can generate a next root.
  if (DEBUG_MOCBS > 0) {
    std::cout << "[DEBUG] ++ MOCBS_TB::_SelectNode curr tree empty, gen next root." << std::endl;
  }
  // clean up current cached high level nodes (to save memory) @2021-12-29 [Engineering Detail]
  _nodes.clear();
  
  if (_NextRoot()) {
    // OPEN cannot be empty now.
    long nid = _open.begin()->second;
    CBSNode* out = &( _nodes[nid] );
    _open.erase(_open.begin());
    if (DEBUG_MOCBS > 0) {
      std::cout << "[DEBUG] ++ MOCBS_TB::_SelectNode node = " << (*out) << std::endl;
    }
    return out; // a node is selected.
  }else{
    return NULL;
  }
};

CBSNode* MOCBS_TB::_NextRoot() {
  if ( !__NextRootUpdateIndex() ) {
    // no next root to be generated...
    return NULL;
  }

  long nid = _GenNodeId();
  _nodes[nid].id = nid;
  CBSNode *out = &(_nodes[nid]); // set as output pointer

  _GetPathSetGetCurr(out);

  //// insert into OPEN.
  _open.insert( std::make_pair(out->g, out->id) );
  _result.n_generated++; // update
  
  return out;
};

bool MOCBS_TB::__NextRootUpdateIndex() {
  if (DEBUG_MOCBS > 2) {
    std::cout << "[DEBUG] ------ MOCBS_TB::__NextRootUpdateIndex, before update, indices : [";
    for (const auto& ii : _initPathIndex) { std::cout << ii.second << ","; }
    std::cout << "]" << std::endl;
  }
  bool generated = false;
  for (auto& ii : _initPathIndex) {
    if ( 1+ ii.second == _initPathId[ii.first].size() ) {
      ii.second = 0; // reach the largest, set to zero, move to the next digit.
      continue;
    }else{
      ii.second++; // move to next.
      generated = true;
      break;
    }
  }
  if (DEBUG_MOCBS > 2) {
    std::cout << "[DEBUG] ------ MOCBS_TB::__NextRootUpdateIndex,  after update, indices : [";
    for (const auto& ii : _initPathIndex) { std::cout << ii.second << ","; }
    std::cout << "], generated = " << int(generated) << std::endl;
  }
  return generated;
};


bool MOCBS_TB::_IfTimeout() {
  auto tnow = std::chrono::steady_clock::now(); // for timing.
  return ( _tlimit < std::chrono::duration<double>(tnow - _t0).count() );
};

bool MOCBS_TB::_SolutionFilter(basic::CostVector& g) {
  // do a naive imipl at first.
  for (auto ii : _result.costs) {
    if ( search::EpsDominance( ii.second, g, _epsilon ) ){
      _result.n_solFiltered++;
      return true;
    }
  }
  return false;

  // // this frontier already do vector projection.
  // return _solFront.Check(g);
};

bool MOCBS_TB::_SuccessorFilter(CBSNode* n, basic::CostVector& g) {
  return false;
};

void MOCBS_TB::_UpdateSolution(CBSNode* n) {
  // _solFront.Update(n->id, n->g);

  if (_result.costs.size() == 0) {
    // runtime to reach the first feasible solution.
    auto tnow = std::chrono::steady_clock::now();
    _result.rt_firstSol = std::chrono::duration<double>(tnow - _t0).count();
    _result.firstSolCost = n->g;
  }

  // filter
  std::vector<long> id_to_delete;
  for (auto& cc : _result.costs){
    // std::cout << " _UpdateSolution, n->g = " << n->g << " sol : " << cc.second << std::endl;
    if (search::EpsDominance(n->g, cc.second, _epsilon)) {
      // std::cout << " _UpdateSolution, deleted ! " << std::endl;
      // std::cout << " input g = " << n->g << " prev sol cost = " << cc.second << std::endl; 
      id_to_delete.push_back(cc.first);
    }
  }
  for (auto& ii : id_to_delete) {
    _result.solus.erase(ii);
    _result.costs.erase(ii);
  }

  // add
  _result.solus[n->id] = n->paths;
  _result.costs[n->id] = n->g;

  if (DEBUG_MOCBS > 0) {
    std::cout << "[DEBUG] *** MOCBS_TB::_UpdateSolution find sol node " << (*n) << std::endl;
  }
  return ;
};

int MOCBS_TB::_LsearchPlan(int ri, std::vector<CBSConstraint*>& cstrs, search::MOSPPResult* out)
{

  auto tnow = std::chrono::steady_clock::now();
  auto time_passed = std::chrono::duration<double>(tnow - _t0).count();
  double remain_time = _tlimit - time_passed;

  if (DEBUG_MOCBS > 0) {
    std::cout << "[DEBUG] *** MOCBS_TB::_LsearchPlan for robot=" << ri 
      << " with remain time " << remain_time << std::endl;
  }

  std::vector< std::vector<long> > ncs, ecs;
  for (auto cstr : cstrs) {
    if (cstr->IsNodeCstr()) {
      std::vector<long> nc;
      nc.emplace_back(cstr->vi);
      nc.emplace_back(cstr->ta);
      ncs.push_back(nc);
      continue;
    }
    if (cstr->IsEdgeCstr()) {
      std::vector<long> ec;
      ec.emplace_back(cstr->vi);
      ec.emplace_back(cstr->vj);
      ec.emplace_back(cstr->ta);
      ecs.push_back(ec);
      continue;
    }
  }

  // auto retFlag = RunMOSIPPGrid(_tempAgentGrids[ri], _vo[ri], -1, -1, _vd[ri], -1, -1, remain_time, _agentCosts[ri], ncs, ecs, out ); 
  auto retFlag = RunBOAstGrid(_tempAgentGrids[ri], _vo[ri], _vd[ri], remain_time, _agentCosts[ri], ncs, ecs, out ); 
  // std::cout << "MO-CBS out = " << *out << std::endl;
  auto tnow2 = std::chrono::steady_clock::now();
  _result.rt_searchLow += std::chrono::duration<double>(tnow2 - tnow).count();
  _result.n_expandLow += out->n_expanded; // update low level expanded.
  return retFlag;

};


int MOCBS_TB::_Lsearch(int ri, CBSNode* n, CBSConstraint* cp)
{
  std::vector<CBSConstraint*> cstrs;
  auto retFlag = _GetAllCstr(ri, n, &cstrs);
  // currently, this retFlag is useless.

  cstrs.push_back(cp); // currently, this pointer is still valid.
  if (DEBUG_MOCBS) {
    std::cout << "[DEBUG] **** MOCBS_TB::_GetAllCstr" << std::endl;
    for (auto & cstr : cstrs) {
      std::cout << "[DEBUG] **** ----- cstr " << (*cstr) << std::endl;
    }
  }
  search::MOSPPResult lowRes;
  auto lowFlag = _LsearchPlan(ri, cstrs, &lowRes);
  // handle different low flag?

  if (DEBUG_MOCBS) {
    std::cout << "[DEBUG] **** MOCBS_TB::_Lsearch, result:" << lowRes << std::endl;
  }

  //// Generate new HL nodes.
  for (const auto& ii : lowRes.paths) { // loop over all idvl paths of agent ri.
    int k = ii.first;

    basic::CostVector newg = n->g - n->costs[ri] +lowRes.costs[k];
    if ( _SolutionFilter(newg) ) {
      continue;
    }
    if ( _SuccessorFilter(n, newg) ) {
      continue;
    }

    long nid = _GenNodeId();
    _nodes[nid] = *n; // make a copy
    _nodes[nid].id = nid;

    // enforce unit time path
    std::vector<long> t2;
    UnitTimePath(lowRes.paths[k], lowRes.times[k], &(_nodes[nid].paths[ri]), &t2); 
      // t2 is useless here. always start from t=0;
    
    _nodes[nid].costs[ri] = lowRes.costs[k];
    _nodes[nid].g = newg;
    _nodes[nid].parent = n->id;
    _nodes[nid].cstr = *cp; // make a copy

    if (DEBUG_MOCBS) {
      std::cout << "[DEBUG] **** MOCBS_TB::_Lsearch, gen node:" << _nodes[nid] << std::endl;
    }

    _open.insert( std::make_pair(_nodes[nid].g, nid) );
    _result.n_generated++; // update, this also include root node.
    _result.n_branch++; // a new branch. not include root node.
    // std::cout << " _result.n_branch = " << _result.n_branch << std::endl;
  }
  // If low level fails to find any path, then lowRes.paths should be empty and 
  // no new high level node will be generated, which is correct.

  return 1;
};


void MOCBS_TB::__InitPathSetUp(int ri) {
  _initPathId[ri] = std::vector<long>();
  for (const auto& iter : _initRes[ri].costs) {
    _initPathId[ri].emplace_back(iter.first);
  }
  _initPathIndex[ri] = 0;
  return;
};

void MOCBS_TB::__InitPathGetCurr(
  int ri, std::vector<long>* path, std::vector<long>* time, basic::CostVector *g)
{
  *path = _initRes[ri].paths[ _initPathId[ri][_initPathIndex[ri]] ] ;
  *time = _initRes[ri].times[ _initPathId[ri][_initPathIndex[ri]] ] ;
  *g = _initRes[ri].costs[ _initPathId[ri][_initPathIndex[ri]] ] ;
  return ;
};

void MOCBS_TB::_GetPathSetGetCurr(CBSNode* out) {

  out->g = basic::CostVector(0, _cdim);
  for (const auto& ii : _initRes) { // loop over all agents.
    int ri = ii.first;
    // std::cout << " >>>> ri = " << ri << std::endl;

    // get path of ri
    std::vector<long> pp, tt;
    basic::CostVector gg;
    __InitPathGetCurr(ri, &pp, &tt, &gg);

    // enforce unit time path.
    out->paths[ri] = std::vector<long>();
    std::vector<long> t2;
    UnitTimePath(pp, tt, &(out->paths[ri]), &t2); // t2 is useless here. always start from t=0;
    
    // update costs
    out->costs[ri] = gg;
    out->g += gg;
    
    // node ID is generated outside this func.
  }
  return ;
};


/////////////////////////////////////////////////////////////////////////
////////////////////// MOCBS_TN /////////////////////
/////////////////////////////////////////////////////////////////////////

MOCBS_TN::MOCBS_TN() {
  std::cout << "[INFO] Constructor - MOCBS_TN" << std::endl;
};

MOCBS_TN::~MOCBS_TN() {};

int MOCBS_TN::_LsearchPlan(int ri, std::vector<CBSConstraint*>& cstrs, search::MOSPPResult* out)
{
  auto tnow = std::chrono::steady_clock::now();
  auto time_passed = std::chrono::duration<double>(tnow - _t0).count();
  double remain_time = _tlimit - time_passed;

  if (DEBUG_MOCBS > 0) {
    std::cout << "[DEBUG] *** MOCBS_N::_LsearchPlan for robot=" << ri 
      << " with remain time " << remain_time << std::endl;
  }

  std::vector< std::vector<long> > ncs, ecs;
  for (auto cstr : cstrs) {
    if (cstr->IsNodeCstr()) {
      std::vector<long> nc;
      nc.emplace_back(cstr->vi);
      nc.emplace_back(cstr->ta);
      ncs.push_back(nc);
      continue;
    }
    if (cstr->IsEdgeCstr()) {
      std::vector<long> ec;
      ec.emplace_back(cstr->vi);
      ec.emplace_back(cstr->vj);
      ec.emplace_back(cstr->ta);
      ecs.push_back(ec);
      continue;
    }
  }

  auto retFlag = RunNAMOAdrstGrid(_tempAgentGrids[ri], _vo[ri], _vd[ri], remain_time, _agentCosts[ri], ncs, ecs, out ); 
  auto tnow2 = std::chrono::steady_clock::now();
  _result.rt_searchLow += std::chrono::duration<double>(tnow2 - tnow).count();
  _result.n_expandLow += out->n_expanded; // update low level expanded.
  return retFlag;
};

/////////////////////////////////////////////////////////////////////////
////////////////////// MOCBS_B /////////////////////
/////////////////////////////////////////////////////////////////////////

MOCBS_B::MOCBS_B() {
  std::cout << "[INFO] Constructor - MOCBS_B" << std::endl;
};

MOCBS_B::~MOCBS_B() {};

bool MOCBS_B::_Init() {

  if (DEBUG_MOCBS > 0) {
    std::cout << "[DEBUG] * MOCBS_B::_Init start " << std::endl;
  }

  _result.n_expandLow = 0.0;
  auto init_t0 = std::chrono::steady_clock::now(); // for timing.

  //// plan individual pareto-optimal path for each agent.
  std::vector<CBSConstraint*> emptySetCstr;
  _result.n_initRoot = 1;
  std::cout << "[INFO] #roots = (";
  for (int ri = 0; ri < _vo.size(); ri++) {
    _initRes[ri] = search::MOSPPResult();
    int ret_flag = _LsearchPlan( ri, emptySetCstr, &(_initRes[ri]) );
    
    _result.n_initRoot *= _initRes[ri].paths.size();
    // std::cout << "_initRes[ri] = " << _initRes[ri] << std::endl;
    std::cout << " " << _initRes[ri].paths.size() << " ";

    if (ret_flag == 0) {
      // TODO, how to handle this exception
      std::cout << "[ERROR] * MOCBS_B::_Init failed at robot " << ri << std::endl;
      throw std::runtime_error( "[ERROR] MOCBS_B::_Init failed !" );
    }
    __InitPathSetUp(ri);
  }

  // generate all roots

  // the first root node. output via a pointer to that root node.
  long nid = _GenNodeId();
  _nodes[nid].id = nid;
  CBSNode* out = &(_nodes[nid]); // set as output pointer
  _GetPathSetGetCurr(out);

  _open.insert( std::make_pair(out->g, out->id) );
  _result.n_generated++; // update

  CBSNode* next_root = _NextRoot();
  while (next_root) {
    next_root = _NextRoot();
  }

  _result.n_branch = 0.0;
  _result.rt_initHeu = std::chrono::duration<double>(std::chrono::steady_clock::now() - init_t0).count();  

  std::cout << "), total = " << _result.n_initRoot << " rt_init = " << _result.rt_initHeu << std::endl;

  return true;
};

CBSNode* MOCBS_B::_SelectNode() {
  if ( !_open.empty() ) {
    long nid = _open.begin()->second;
    CBSNode* out = &( _nodes[nid] );
    _open.erase(_open.begin());
    if (DEBUG_MOCBS > 0) {
      std::cout << "[DEBUG] ++ MOCBS_B::_SelectNode node = " << (*out) << std::endl;
    }
    return out; // a node is selected.
  }
  // if _open is empty, no way to continue.
  return NULL;
};

/////////////////////////////////////////////////////////////////////////
////////////////////// MOCBS_N /////////////////////
/////////////////////////////////////////////////////////////////////////

MOCBS_N::MOCBS_N() {
  std::cout << "[INFO] Constructor - MOCBS_N" << std::endl;
};

MOCBS_N::~MOCBS_N() {};

int MOCBS_N::_LsearchPlan(int ri, std::vector<CBSConstraint*>& cstrs, search::MOSPPResult* out)
{

  auto tnow = std::chrono::steady_clock::now();
  auto time_passed = std::chrono::duration<double>(tnow - _t0).count();
  double remain_time = _tlimit - time_passed;

  if (DEBUG_MOCBS > 0) {
    std::cout << "[DEBUG] *** MOCBS_N::_LsearchPlan for robot=" << ri 
      << " with remain time " << remain_time << std::endl;
  }

  std::vector< std::vector<long> > ncs, ecs;
  for (auto cstr : cstrs) {
    if (cstr->IsNodeCstr()) {
      std::vector<long> nc;
      nc.emplace_back(cstr->vi);
      nc.emplace_back(cstr->ta);
      ncs.push_back(nc);
      continue;
    }
    if (cstr->IsEdgeCstr()) {
      std::vector<long> ec;
      ec.emplace_back(cstr->vi);
      ec.emplace_back(cstr->vj);
      ec.emplace_back(cstr->ta);
      ecs.push_back(ec);
      continue;
    }
  }

  auto retFlag = RunNAMOAdrstGrid(_tempAgentGrids[ri], _vo[ri], _vd[ri], remain_time, _agentCosts[ri], ncs, ecs, out ); 
  auto tnow2 = std::chrono::steady_clock::now();
  _result.rt_searchLow += std::chrono::duration<double>(tnow2 - tnow).count();
  _result.n_expandLow += out->n_expanded; // update low level expanded.
  return retFlag;
};


// /////////////////////////////////////////////////////////////////////////
// ////////////////////// MOCBS_TSF /////////////////////
// /////////////////////////////////////////////////////////////////////////

// MOCBS_TSF::MOCBS_TSF() {
//   std::cout << "[INFO] Constructor - MOCBS_TSF" << std::endl;
// };

// MOCBS_TSF::~MOCBS_TSF() {};

// bool MOCBS_TSF::_SuccessorFilter(CBSNode* n, basic::CostVector& g) {
//   // std::cout << " MOCBS_TSF::_SuccessorFilter ! " << std::endl;
//   // the generated successor's cost vector g must be dominated by the parent's cost vector !
//   if ( search::EpsDominance( n->g, g ) ){
//     return false; // should keep it, don't filter it.
//   }
//   return true; // filter it !
// };


} // end namespace mapf
} // end namespace rzq
