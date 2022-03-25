
/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/

#include "union_find.hpp"

namespace rzq{
namespace basic{


/**
 * @brief Find operation in Union-find data structure. 
 *  Assume a is in dic. No safety check. You should do that outside.
 */
template<typename DataType>
DataType UFFind(std::unordered_map<DataType,DataType>* dic, DataType a) {
  // if (dic->find(a) == dic->end()) {return -1;}
  while ((*dic)[a] != a) {
    (*dic)[a] = (*dic)[(*dic)[a]]; // path compression
    a = (*dic)[a];
  }
  return a;
};
/**
 * @brief Union the sets that contains element a and b.
 *  Returned value int indicates the running status of this func.
 */
template<typename DataType>
int UFUnion(std::unordered_map<DataType,DataType>* dic, const DataType& a, const DataType& b) {
  if (dic->find(a) == dic->end()) {return -1;}
  if (dic->find(b) == dic->end()) {return -2;}
  auto rooti = UFFind(dic,a);
  auto rootj = UFFind(dic,b);
  if (rooti == rootj){
    return 0;
  }
  else{
    if (rooti > rootj){
      (*dic)[rootj] = rooti;
    }else{
      (*dic)[rooti] = rootj;
    }
    return 1;
  }
};

void CompileHelperUnionFind() {
  std::unordered_map<int,int> a1;
  UFFind(&a1,1);
  UFUnion(&a1,1,2);
  std::unordered_map<long,long> a2;
  UFFind(&a2,1l);
  UFUnion(&a2,1l,2l);
  std::unordered_map<short,short> a3;
  UFFind(&a3,short(1));
  UFUnion(&a3,short(1),short(2));
  std::unordered_map<std::string,std::string> a4;
  UFFind(&a4,std::to_string(1));
  UFUnion(&a4,std::to_string(1),std::to_string(2));
  return;
};

} // end namespace basic
} // end namespace rzq
