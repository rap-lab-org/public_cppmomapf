/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/

#include "avltree.hpp"
#include "graph.hpp"

#include "signal.h"

namespace rzq{
namespace basic{

std::ostream& operator<<(std::ostream& os, const AVLNode& n) {
  os << "AVLNode(id:" << n.id << ",hasLeft:" << int(n.left!=NULL) << ",hasRight:" << int(n.right!=NULL) << ",h:" << n.h << ")";
  return os;
};

AVLNode* NewAVLNode(long id) {
  AVLNode *n = new AVLNode();
  n->id = id;
  n->left = NULL;
  n->right = NULL;
  n->h = 1;
  return n;
};

AVLNode *RightRotate(AVLNode *y) {
  // find related data
  AVLNode *x = y->left;
  AVLNode *t2 = x->right;

  // rotate
  x->right = y;
  y->left = t2;
  // update height
  y->h = Max( H(y->left), H(y->right) ) + 1;
  x->h = Max( H(x->left), H(x->right) ) + 1;
  return x;
};

AVLNode *LeftRotate(AVLNode *x) {
  // find related data
  AVLNode *y = x->right;
  AVLNode *t2 = y->left;

  // rotate
  y->left = x;
  x->right = t2;
  // update height
  x->h = Max( H(x->left), H(x->right) ) + 1;
  y->h = Max( H(y->left), H(y->right) ) + 1;

  return y;
};

int GetBalanceFactor(AVLNode *n) {
  if (n == NULL) {return 0;}
  return H(n->left) - H(n->right);
};


template <typename DataType>
AVLTree<DataType>::AVLTree() {};


template <typename DataType>
AVLTree<DataType>::~AVLTree() {
  _deleteAll(_root); // need to delete/free all nodes.
};
  
template <typename DataType>
void AVLTree<DataType>::Add(DataType k, long id) {
  _root = _insert(_root, k, id);
  return;
};

template <typename DataType>
void AVLTree<DataType>::Print() {
  std::cout << "AVL-tree:{";
  _PreOrder(_root);
  std::cout << "}" << std::endl;
  return;
};

template <typename DataType>
AVLNode AVLTree<DataType>::Find(DataType k) {
  AVLNode* p = _find(_root, k);
  if (p == NULL) {
    return AVLNode();
  }else{
    return *p; // make a copy to return
  }
};

template <typename DataType>
int AVLTree<DataType>::FindMaxLess(DataType k, DataType *out, bool if_equal, long *out_id) {
  long ref = -1;
  this->_findMaxLess(_root, k, &ref, if_equal);
  if (ref < 0) {
    return 0;
  }
  *out = _key[ref];
  if (out_id != NULL) {
    *out_id = ref; // copy tree-node ID, if needed.
  }
  return 1;
};

template <typename DataType>
int AVLTree<DataType>::FindMinMore(DataType k, DataType *out, bool if_equal, long *out_id) {
  long ref = -1;
  _findMinMore(_root, k, &ref, if_equal);
  if (ref < 0) {
    return 0;
  }
  *out = _key[ref];
  if (out_id != NULL) {
    *out_id = ref; // copy tree-node ID, if needed.
  }
  return 1;
};

template <typename DataType>
void AVLTree<DataType>::Delete(DataType k) {
  _root = _delete(_root, k);
  return;
};

template <typename DataType>
void AVLTree<DataType>::Clear() {
  _key.clear(); // compare the keys of two nodes, (not their id.)
  _deleteAll(_root); // need to delete/free all nodes.
  _root = NULL; 
  _id_gen = 0;
  _size = 0;
  return ;
};

template <typename DataType>
size_t AVLTree<DataType>::Size() const {
  return _size;
}; 

template <typename DataType>
void AVLTree<DataType>::ToSortedVector(
  std::vector<DataType> *out, std::vector<long> *out_id, std::unordered_set<long> *skip_set)
{
  _InOrder2Vec(_root, out, out_id, skip_set);
  return;
};

template <typename DataType>
void AVLTree<DataType>::_PreOrder(AVLNode *n) {
  if(n != NULL) {
    std::cout << "[k:" << _key[n->id] << ",h:" << n->h << ",id:" << n->id << "],";
    _PreOrder(n->left);
    _PreOrder(n->right);
  }
return;
};

template <typename DataType>
void AVLTree<DataType>::_InOrder2Vec(
  AVLNode *n, std::vector<DataType> *out, std::vector<long> *out_id, 
  std::unordered_set<long> *skip_set) 
{
  if (n == NULL) {
    return;
  }

  _InOrder2Vec(n->left, out, out_id, skip_set);

  if ( (skip_set == NULL) || 
       (skip_set->find(n->id) == skip_set->end()) )
  {
    if (out != NULL) {
      out->push_back(_key[n->id]);
    }
    if (out_id != NULL) {
      out_id->push_back(n->id);
    }
  }

  _InOrder2Vec(n->right, out, out_id, skip_set);

  return ;
};


template <typename DataType>
AVLNode* AVLTree<DataType>::_find(AVLNode* n, DataType k) {
  if (n == NULL) {
    return n;
  }
  if (k < _key[n->id]) {
    return _find(n->left, k);
  }else if (k > _key[n->id]) {
    return _find(n->right, k);
  }else {
    return n; // find it !
  }
  throw std::runtime_error( "[WARNING] AVL-tree _find reaches the end !?" );
};


template <typename DataType>
void AVLTree<DataType>::_findMaxLess(AVLNode* n, DataType k, long* ref, bool if_equal) {
  if (n == NULL) {
    return;
  }
  if (_key[n->id] < k) {
    bool temp = (*ref == -1) || (_key[n->id] > _key[*ref]);
    if (temp) {
      *ref = n->id; // update ref.
    }
    this->_findMaxLess(n->right, k, ref, if_equal);
    return;
  }else if (_key[n->id] > k) { 
    this->_findMaxLess(n->left, k, ref, if_equal);
    return;
  }else{ // _key[n->id] == k
    if (if_equal) {
      *ref = n->id;
      return; // no need to go deeper...
    } else { // search over the leftt tree.
      this->_findMaxLess(n->left, k, ref, if_equal);
      return;
    }
  }
  throw std::runtime_error( "[WARNING] AVL-tree _findMaxLess reaches the end !?" );
};

template <typename DataType>
void AVLTree<DataType>::_findMinMore(AVLNode* n, DataType k, long* ref, bool if_equal) {
  if (n == NULL) {
    return;
  }
  if (_key[n->id] > k) {
    bool temp = ((*ref == -1) || (_key[n->id] < _key[*ref]));
    if (temp) {
      *ref = n->id; // update ref.
    }
    _findMinMore(n->left, k, ref, if_equal);
    return;
  }else if (_key[n->id] < k) { 
    _findMinMore(n->right, k, ref, if_equal);
    return;
  }else{ // _key[n->id] == k
    if (if_equal) {
      *ref = n->id;
      return; // no need to go deeper...
    } else { // search over the leftt tree.
      _findMinMore(n->right, k, ref, if_equal);
      return;
    }
  }
  throw std::runtime_error( "[WARNING] AVL-tree _findMinMore reaches the end !?" );
};

template <typename DataType>
AVLNode* AVLTree<DataType>::_insert(AVLNode* n, DataType k, long id0) {
  // BST insertion.

  if (n == NULL) {
    auto nn = NewAVLNode(id0);
    if (id0 < 0) {
      nn->id = _IdGen();
    }
    _key[nn->id] = k; // store key
    _size++;

    return nn;
  }

  if (k < _key[n->id]) {
    n->left = _insert(n->left, k, id0);
  }else if (k > _key[n->id]) {
    n->right = _insert(n->right, k, id0);
  }else {
    // throw std::runtime_error( "[WARNING] duplicated key in AVL tree !?" );
    return n; // duplicated key, ignore, return the old one.
  }
  // update height
  n->h = Max( H(n->left),H(n->right) ) + 1;
  // balance factor
  int b = GetBalanceFactor(n);

  // four cases
  if (b > 1 && (n->left != NULL) && k < _key[n->left->id]) {
    auto temp = RightRotate(n);
    if (DEBUG_AVLTREE) { _verifyTree(n);}
    return temp;
  }
  if (b < -1 && (n->right != NULL) && k > _key[n->right->id]) {
    auto temp = LeftRotate(n);
    if (DEBUG_AVLTREE) { _verifyTree(n);}
    return temp;
  }
  if (b > 1 && (n->left != NULL) && k > _key[n->left->id]) {
    n->left = LeftRotate(n->left);
    auto temp = RightRotate(n);
    if (DEBUG_AVLTREE) { _verifyTree(n);}
    return temp;
  }
  if (b < -1 && (n->right != NULL) && k < _key[n->right->id]) {
    n->right = RightRotate(n->right);
    auto temp = LeftRotate(n);
    if (DEBUG_AVLTREE) { _verifyTree(n);}
    return temp;
  }
  if (DEBUG_AVLTREE) { _verifyTree(n);}
  return n;
};

template <typename DataType>
AVLNode* AVLTree<DataType>::_findMin(AVLNode* n) {
  if (n == NULL) {return NULL;}
  AVLNode* current = n;
  // find the left-most leaf
  while (current->left != NULL)
    current = current->left;
  return current;
};

template <typename DataType>
AVLNode* AVLTree<DataType>::_delete(AVLNode* n, DataType k) {
  // BST delete
  if (n == NULL) {
    return n;
  }
  if ( k < _key[n->id] ) {
    n->left = _delete(n->left, k);
  } else if( k > _key[n->id] ) {
    n->right = _delete(n->right, k);
  } else { // this is the node to be deleted.
    if( (n->left == NULL) ||
        (n->right == NULL) ) 
    { // one child or no child.
      AVLNode *child = n->left ? n->left : n->right;
      if (child == NULL) { // no child
        delete n;
        _size--;
        n = NULL;
      } else {// one child case
        *n = *child; // copy the non-empty child...
        if (child == n->left) {n->left = NULL;}
        else {n->right = NULL;}
        delete child;
        _size--;
        child = NULL;
      }
    } else { // two children: get the smallest in the right subtree
      AVLNode* temp = _findMin(n->right);
      n->id = temp->id; // copy
      // Delete the inorder successor
      n->right = _delete(n->right, _key[temp->id]);
    }
  }

  if (DEBUG_AVLTREE) {_verifyLessThan(n);}

  if (n == NULL) { // tree is empty now...
    return n;
  }
  // Update height
  n->h = 1 + Max(H(n->left), H(n->right));
  // Balancing
  int b = GetBalanceFactor(n);
  if ( (b>1) && (GetBalanceFactor(n->left) >= 0) ) {
    auto temp = RightRotate(n);
    return temp;
  }
  if ( (b>1) && (GetBalanceFactor(n->left) < 0) ) {
    n->left = LeftRotate(n->left);
    auto temp = RightRotate(n);
    return temp;
  }
  if ( (b<-1) && (GetBalanceFactor(n->right) <= 0) ) {
    auto temp = LeftRotate(n);
    return temp;
  } 
  if ( (b<-1) && (GetBalanceFactor(n->right) > 0) ) {
    n->right = RightRotate(n->right);
    auto temp = LeftRotate(n);
    return temp;
  }
  return n;
}

template <typename DataType>
void AVLTree<DataType>::_deleteAll(AVLNode* n) {
  // BST delete
  if (n == NULL) {
    return ;
  }
  _deleteAll(n->left);
  _deleteAll(n->right);
  delete n;
  return;
};


template <typename DataType>
void AVLTree<DataType>::_verifyTree(AVLNode* n) {
  if (n == NULL) {return;}

  _verifyLessThan(n);

  int b = GetBalanceFactor(n);
  if ((b < -1) || (b > 1)) {
    std::cout << "[DEBUG] current node = " << (*n) << std::endl;
    throw std::runtime_error("[ERROR] AVLTree is not balanced!");
  }

  if (n->left) {
    int b = GetBalanceFactor(n->left);
    if ((b < -1) || (b > 1)) {
      std::cout << "[DEBUG] n->left node = " << (*n->left) << std::endl;
      throw std::runtime_error("[ERROR] AVLTree is not balanced!");
    }
  }

  if (n->right) {
    int b = GetBalanceFactor(n->right);
    if ((b < -1) || (b > 1)) {
      std::cout << "[DEBUG] n->right node = " << (*n->right) << std::endl;
      throw std::runtime_error("[ERROR] AVLTree is not balanced!");
    }
  }
  return;
};

template <typename DataType>
void AVLTree<DataType>::_verifyLessThan(AVLNode* n) {
  if (n == NULL) {return;}
  if ( (n->left && _key[n->id] < _key[n->left->id]) || 
       (n->right && _key[n->id] > _key[n->right->id]) )
  {
    std::cout << " _key[n] = " << _key[n->id] << ", ";
    if (n->left) {std::cout << "_key[n->left] = " << _key[n->left->id] << ", ";}
    if (n->right) {std::cout << "_key[n->right] = " << _key[n->right->id] << ", ";}
    std::cout << std::endl;
    throw std::runtime_error("[ERROR] AVL-tree is not a binary search tree !?");
  }
  return;
};

template<typename DataType>
void AVLTreeCompileHelper() {
  DataType a;
  AVLTree<DataType> t;
  t.Add(a);
  t.Size();
  t.Print();
  DataType b;
  t.Find(a);
  t.FindMaxLess(a, &b, false, NULL);
  t.FindMinMore(a, &b, false, NULL);
  t.Delete(a);
  t.Clear();
};

void AVLTreeInstantiation(){
  AVLTreeCompileHelper<rzq::basic::CostVector>();
  AVLTreeCompileHelper<int>();
  AVLTreeCompileHelper<short>();
  AVLTreeCompileHelper<long>();
  AVLTreeCompileHelper<float>();
  AVLTreeCompileHelper<double>();
}


template class AVLTree<rzq::basic::CostVector>;
template class AVLTree<int>;
template class AVLTree<short>;
template class AVLTree<long>;
template class AVLTree<float>;
template class AVLTree<double>;


} // namespace basic
} // namespace rzq
