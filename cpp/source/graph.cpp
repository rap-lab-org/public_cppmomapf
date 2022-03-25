
/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/


#include "graph.hpp"

namespace rzq{
namespace basic{


CostVector::CostVector(){};

CostVector::CostVector(long val, size_t dim) {
  this->resize(dim);
  for (size_t i = 0; i < dim; i++){
    this->at(i) = val;
  }
  return;
};

CostVector::CostVector(const std::vector<long>& in) {
  this->resize(in.size());
  for (size_t i = 0; i < in.size(); i++){
    this->at(i) = in.at(i);
  }
  return;
};

CostVector CostVector::operator+(const CostVector& v) {
  CostVector out = *this;
  for (size_t i = 0; i < v.size(); i++) {
    out[i] += v[i];
  }
  return out;
};

CostVector& CostVector::operator+=(const CostVector& rhs){
  for (size_t i = 0; i < rhs.size(); i++) {
    this->at(i) += rhs[i];
  }
  return *this;
}

CostVector CostVector::operator-(const CostVector& v) {
  CostVector out = *this;
  for (size_t i = 0; i < v.size(); i++) {
    out[i] -= v[i];
  }
  return out;
};

CostVector CostVector::operator*(const long& k) {
  CostVector out = *this;
  for (size_t i = 0; i < this->size(); i++) {
    out[i] *= k;
  }
  return out;
};

CostVector CostVector::ElemWiseMin(const CostVector& rhs) {
  CostVector out = *this;
  for (size_t i = 0; i < this->size(); i++) {
    if (rhs[i] < out[i]) {
      out[i] = rhs[i];
    }
  }
  return out;
};

std::string CostVector::ToStr() const {
  std::string s = "[";
  for (auto a : (*this) ) {
    s += std::to_string(a) + ",";
  }
  s += "]";
  return s;
};

std::ostream& operator<<(std::ostream& os, const CostVector& c) {
  os << c.ToStr();
  return os;
};

// ##########

Grid::Grid() {};

Grid::~Grid() {
  // std::cout << "Grid destructor done" << std::endl;
};

void Grid::Resize(size_t r, size_t c, int val) {
  this->resize(r);
  for (auto iter = this->begin(); iter != this->end(); iter++){
    iter->resize(c, val);
  }
  return;
};

size_t Grid::GetRowNum() const {
  return this->size();
};

size_t Grid::GetColNum() const {
  if (this->size() == 0) {return 0;}
  return this->at(0).size();
};

void Grid::Set(size_t r, size_t c, int val) {
  this->at(r).at(c) = val;
};

int Grid::Get(size_t r, size_t c) {
  return this->at(r).at(c);
};


GridkConn::GridkConn() {};

GridkConn::~GridkConn() {
};

void GridkConn::Init(Grid grid, std::vector<Grid> cvecs) {
  // grid and dimensions
  _grid = grid;
  _nc = _grid.GetColNum();
  _nr = _grid.GetRowNum();
  // costs
  _cvecs = cvecs;
  _cdim = 0;
  if (cvecs.size() != 0){
    _cdim = _cvecs.size();
    // re-organize cvecs.
    _cvecs2.resize(_nr);
    for (size_t y = 0; y < _nr; y++){
      _cvecs2[y].resize(_nc);
      for (size_t x = 0; x < _nc; x++){
        _cvecs2[y][x].resize(_cdim);
      }
    }
    for (size_t y = 0; y < cvecs.begin()->size(); y++) {
      for (size_t x = 0; x < cvecs.begin()->begin()->size(); x++) {
        CostVector c;
        for (size_t k = 0; k < _cvecs.size(); k++) {
          c.push_back(_cvecs[k][y][x]);
        }
        _cvecs2[y][x] = c;
      }
    }
  }
  // actions
  _actions.clear();
  _actions.push_back(std::vector<int>({-1,0}));
  _actions.push_back(std::vector<int>({1,0}));
  _actions.push_back(std::vector<int>({0,1}));
  _actions.push_back(std::vector<int>({0,-1}));
  return;
};

void GridkConn::SetActionSet(std::vector< std::vector<int> > actions) {
  _actions = actions;
};

bool GridkConn::HasNode(long v) {
  auto ny = y(v);
  auto nx = x(v);
  if ((ny >= 0) && (ny < _nr) && (nx >= 0) && (nx < _nc)) { // within border
    return true;
  }
  return false;
};

std::unordered_set<long> GridkConn::GetSuccs(long v) {
  std::unordered_set<long> out;
  for (auto iter = _actions.begin(); iter != _actions.end(); iter++){
    int ny = y(v)+iter->at(0);
    int nx = x(v)+iter->at(1);
    // std::cout << " GridkConn::GetSuccs, ny = " << ny << " nx = " << nx << std::endl;
    if ((ny >= 0) && (ny < _nr) && (nx >= 0) && (nx < _nc)) { // within border
      if ( _grid[ny][nx] == 0 ){ // non-obstacle
        // std::cout << " GridkConn::GetSuccs, insert into output " << std::endl;
        out.insert( this->v(ny,nx) );
      }
    }
  }
  return out;
};

std::unordered_set<long> GridkConn::GetPreds(long v) {
  return GetSuccs(v); // due to symmetry.
};

CostVector GridkConn::GetCost(long u, long v) {
  // u is useless! only for grid like graph.
  return _cvecs2[y(v)][x(v)];
};

size_t GridkConn::GetCostDim() {
  return _cdim;
};

long GridkConn::GetGridValue(long u) {
  return _grid[y(u)][x(u)];
};

long GridkConn::v(int y, int x) {
  return x + y * _nc;
};
int GridkConn::y(long v) {
  return v / _nc;
};
int GridkConn::x(long v) {
  return v % _nc;
};

} // end namespace basic
} // end namespace rzq
