
/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/

#include "momapf_util.hpp"

namespace rzq{
namespace mapf{

std::ostream& operator<<(std::ostream& os, const PathSet& ps) {
  os << "{";
  for (auto& kk : ps) {
    os << "agent:" << kk.first << ",path:{";
    for (auto& jj : kk.second) {
      os << jj << ",";
    }
    os << "};";
  }
  os << "}.";
  return os;
};

std::ostream& operator<<(std::ostream& os, const MOMAPFResult& res) {
  os << "MOMAPFResult{find_all_pareto:" << int(res.find_all_pareto) << ",find(" << res.costs.size() << ")solutions:";
  for (const auto iter : res.solus) {
    std::cout << "<ID:" << iter.first << ",cost:" << res.costs.at(iter.first) << ",jointPath:" << iter.second << ">;"; 
  }
  return os;

};

} // end namespace mapf
} // end namespace rzq
