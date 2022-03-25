
/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/

#include <iostream>
#include "api.hpp"


int Example();

int main() {
  Example();
  return 0;
};

int Example(){

  std::cout << "####### test_mocbs.cpp - Example() Begin #######" << std::endl;

  // ------------static environment, 4x4, obstacle at (y=0,x=2), two robots---------
  //   x:0  1  2  3
  // y ---------------
  // 0 | * R1  #  *  |
  // 1 | R2 *  *  *  |
  // 2 | *  *  *  G2 |
  // 3 | *  *  G1 *  |
  //   ---------------
  //  Note: *=free, #=obstacle, R1,R2 = robot start, G1,G2 = robot goals
  // -------------------------------------------------------------------------------
  rzq::basic::Grid static_world; // static obstacles appears in this 2d grid.
  int r = 4; // rows (y)
  int c = 4; // columns (x)
  static_world.Resize(r,c);
  static_world.Set(0,0,1); // set grid[y=0,x=0] = 1, a static obstacle.
  static_world.Set(0,2,1); // set grid[y=0,x=2] = 1, a static obstacle.


  // declare cost structure. M=2

  // ------------the first component of all cost vectors---------
  //   x:0  1  2  3
  // y ---------------
  // 0 | 10 10 #  10 |
  // 1 | 10 10 10  1 |
  // 2 | 10 10 10 10 |
  // 3 | 10 10 10 10 |
  //   ---------------
  //  Note: to simplify implementation, 
  //        the cost of an edge is the cost at the target node of that edge,
  //        i.e. the cost of an edge (u,v) = the cost defined at node v. 
  // -------------------------------------------------------------------------------
  rzq::basic::Grid cost1;
  cost1.Resize(r,c,10); // all one
  cost1.Set(1,3,1); // cost1[y=1,x=2]=1

  // ------------the second component of all cost vectors---------
  //   x:0  1  2  3
  // y ---------------
  // 0 | 10 10 #  10 |
  // 1 | 10 10 10 10 |
  // 2 | 10 10  1 10 |
  // 3 | 10 10 10 10 |
  //   ---------------
  //  Note: to simplify implementation, 
  //        the cost of an edge is the cost at the target node of that edge,
  //        i.e. the cost of an edge (u,v) = the cost defined at node v. 
  // -------------------------------------------------------------------------------
  rzq::basic::Grid cost2; // the second components of cost vectors.
  cost2.Resize(r,c,10); // all three
  cost2.Set(2,2,1); // cost2[y=2,x=2]=1

  // group two cost grid together.
  std::vector<rzq::basic::Grid> cost_grids; // cost vectors (implemented at nodes rather than edges), arrival cost at a node.
  cost_grids.push_back(cost1);
  cost_grids.push_back(cost2);

  // create a graph that contains both static world and cost grids. This is the input to the planner.
  rzq::basic::GridkConn g;
  g.Init(static_world, cost_grids);

  // prepare to run MO-CBS
  std::vector<long> starts({1,4}); // node id = y*NumX + x; e.g. (y=1)*4 + (x=0) = 4
                                   // the length of starts shows how many agents are there.
  std::vector<long> goals({14,11});

  rzq::mapf::MOMAPFResult res;

  std::vector<long> wait_cost; // empty wait_cost vector. wait_cost to the cost vector at that node;
                               // if wait_cost.size() != 0, will use this wait_cost as the wait cost.

  int use_tree_by_tree = 1; // 1=use tree_by_tree, 0=no use tree_by_tree.

  double time_limit = 300; // runtime limit.

  // run MO-CBS
  rzq::mapf::RunMOCBS(g, starts, goals, &res, time_limit, wait_cost, use_tree_by_tree);

  // result data structure is defined in momapf_util.hpp, it contains a lot of fields.
  std::cout << " result = " << res << std::endl;

  std::cout << "####### test_mocbs.cpp - Example() End #######" << std::endl;

  return 1;
};
