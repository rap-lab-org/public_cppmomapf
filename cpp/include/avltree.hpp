/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/

#ifndef ZHONGQIANGREN_BASIC_AVLTREE_H_
#define ZHONGQIANGREN_BASIC_AVLTREE_H_

#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace rzq{
namespace basic{

#define DEBUG_AVLTREE 0

/**
 * @brief AVL-tree node.
 */
struct AVLNode {
  long id = -1;
  AVLNode *left = NULL;
  AVLNode *right = NULL;
  size_t h = 0; // height
};
/**
 * 
 */
std::ostream& operator<<(std::ostream& os, const AVLNode& n) ;
/**
 * @brief max value of two size_t.
 */
inline size_t Max(size_t a, size_t b) {return (a > b) ? a: b;};
/**
 * @brief return the height value of a tree node.
 */
inline size_t H(AVLNode *n) {
  if (n == NULL) { return 0; }
  return n->h;
};
/**
 * @brief create a new tree node.
 */
AVLNode* NewAVLNode(long id) ;
/**
 * @brief right rotate an AVL tree node.
 */
AVLNode *RightRotate(AVLNode *y) ;
/**
 * @brief left rotate an AVL tree node.
 */
AVLNode *LeftRotate(AVLNode *x) ;
/**
 * @brief get the balance factor of a tree node.
 */
int GetBalanceFactor(AVLNode *n) ;
/**
 * @brief AVL-tree, with some extended APIs.
 */
template <typename DataType>
class AVLTree {
public:
  /**
   * @brief AVL-tree constructor
   */
  AVLTree() ;
  /**
   * @brief AVL-tree destructor
   */
  virtual ~AVLTree();
  /**
   * @brief Add an object (i.e. key) into the tree.
   */
  virtual void Add(DataType k, long id = -1) ;
  /**
   * @brief print the tree (in pre-order)
   */
  virtual void Print() ;
  /**
   * @brief return the node that contains key k. If not has, return a node with height 0.
   */
  virtual AVLNode Find(DataType k) ;
  /**
   * @brief return the node with the largest key that is smaller than k.
   * if_equal=true, FindMax(LessOrEqual), if_equal=false, FindMax(StrictLess).
   */
  virtual int FindMaxLess(DataType k, DataType *out, bool if_equal=false, long *out_id=NULL) ;
  /**
   * @brief return the node with the smallest key that is larger than k.
   */
  virtual int FindMinMore(DataType k, DataType *out, bool if_equal=false, long *out_id=NULL) ;
  /**
   * @brief Delete from tree.
   */
  virtual void Delete(DataType k) ;
  /**
   * @brief clear everything. reset.
   */
  virtual void Clear() ;
  /**
   * @brief number of nodes.
   */
  virtual size_t Size() const; 
  /**
   * @brief recursive. traverse the tree in-order and output a vector (sorted)
   * out_id is an optional arg for output the IDs of the data stored in the tree.
   * skip_set = skip the node in this set.
   */
  virtual void ToSortedVector(
    std::vector<DataType> *out, std::vector<long> *out_id=NULL, std::unordered_set<long> *skip_set=NULL) ;

protected:
  /**
   * @brief (Protected) recursive. traverse the tree in pre-order and print..
   */
  virtual void _PreOrder(AVLNode *n) ;
  /**
   * @brief (Protected) recursive. traverse the tree in in-order and generate a vector (sorted) ...
   * out_id is an optional arg for output the IDs of the data stored in the tree.
   */
  virtual void _InOrder2Vec(
    AVLNode *n, std::vector<DataType> *out=NULL, std::vector<long> *out_id=NULL, 
    std::unordered_set<long> *skip_set=NULL) ;
  /**
   * @brief (Protected) recursive. look up a key, return NULL if non-exist..
   */
  virtual AVLNode* _find(AVLNode* n, DataType k) ;
  /**
   * @brief (Protected) recursive. find the max key within the tree that is less than k..
   */
  virtual void _findMaxLess(AVLNode* n, DataType k, long* ref, bool if_equal) ;
  /**
   * @brief (Protected) recursive. find the min key within the tree that is larger than k..
   */
  virtual void _findMinMore(AVLNode* n, DataType k, long* ref, bool if_equal) ;
  /**
   * @brief (Protected) recursive. insert a key..
   */
  virtual AVLNode* _insert(AVLNode* n, DataType k, long id0) ;
  /**
   * @brief (Protected) iterative. find the left-most leaf (i.e. the min). Aux for _delete()
   */
  virtual AVLNode* _findMin(AVLNode* n) ;
  /**
   * @brief (Protected) recursive. delete a key.
   */
  virtual AVLNode* _delete(AVLNode* n, DataType k) ;
  /**
   * 
   */
  virtual void _deleteAll(AVLNode* n) ;
  /**
   * @brief (Protected) inline. generate an ID (internal usage) for a key (object to be stored in this tree)..
   */
  inline long _IdGen(){ return _id_gen++; } ;
  /**
   * @brief (Protected) for debug purposes. comment out in production.
   */
  virtual void _verifyTree(AVLNode* n);
  virtual void _verifyLessThan(AVLNode* n);

  std::unordered_map<long, DataType> _key; // compare the keys of two nodes, (not their id.)
  AVLNode* _root = NULL;
  long _id_gen=0;
  size_t _size = 0;
};



} // namespace basic
} // namespace rzq

#endif  // ZHONGQIANGREN_BASIC_AVLTREE_H_