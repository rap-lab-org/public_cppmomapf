
/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/

#ifndef ZHONGQIANGREN_SEARCH_NAMOADR_ST_H_
#define ZHONGQIANGREN_SEARCH_NAMOADR_ST_H_

#include "boa_st.hpp"

namespace rzq{
namespace search{

#define DEBUG_NAMOADRST 0

basic::CostVector _proj(basic::CostVector v); // remove the first dimension. projection

/**
 * @brief store all non-dominated vectors. Do linear scan for checking and updating.
 */
class FrontierLinear{
public:
  /**
   * @brief
   */
  FrontierLinear() ;
  virtual ~FrontierLinear() ;
  virtual bool Check(basic::CostVector g) ;
  virtual bool drCheck(basic::CostVector g) ;
  virtual bool Remove(LabelST& l) ;
  virtual void Add(LabelST& l) ; // this directly add without doing dominance check (filtering).
  virtual void Filter(LabelST& l, std::unordered_set<long> *deleted=NULL); // use l to filter existing labels.
  virtual void Update(LabelST& l, std::unordered_set<long> *deleted=NULL) ; // Filter + Add

  std::unordered_map<long, LabelST> labels; // just g-vec is needed, TODO, this can be improved.
};


/**
 * @brief an interface / base class, use its pointer
 */
class NAMOAdrst : public BOAst {
public:
  NAMOAdrst() ;

  virtual ~NAMOAdrst() ;

  virtual int Search(long vo, long vd, double time_limit) ;

protected:

  virtual void _Gop2Gcl(LabelST& l) ;

  virtual void _AddSols(LabelST& l) ;

  virtual void _FilterOpenF(LabelST& l) ;

  virtual void _Expand(LabelST& l) ;

  virtual bool _GopGclDomCheck(LabelST& l) ;

  virtual bool _SolDomCheck(basic::CostVector& f) ;

  virtual void _FilterGopAndOpenAndAdd(LabelST& l) ;

  virtual void _FilterGcl(LabelST& l) ;

  std::unordered_map< std::string, FrontierLinear > _g_cl;
  std::unordered_map< std::string, FrontierLinear > _g_op; 

  std::vector<long> _sol_label_ids; // to save solution label ids at vd
};


int RunNAMOAdrstGrid(
  basic::GridkConn& g, long vo, long vd, double time_limit, basic::CostVector& wait_cost,
  std::vector< std::vector<long> >& ncs, std::vector< std::vector<long> >& ecs, MOSPPResult* res) ;


} // end namespace search
} // end namespace rzq


#endif  // ZHONGQIANGREN_SEARCH_NAMOADR_ST_H_
