#ifndef __LABEL_H__
#define __LABEL_H__

#include "ltl.h"
#include "rrtstar.h"

varray_t* construct_tree(int fix, iarray_t* accept, vertex_t* init, int num_runs, float*** obst, int numAPs, int num_robs, int num_regions, carray_t* alpha, float*** goal, BA_t* B, float*** environ, int num_obstacles, int obstacle_size, int region_size);
void writePlotFile(varray_t* endpts, iarray_t* accept, int call);
void writeLeastCostPlotFile(vertex_t* min, int call);
varray_t* findPath(varray_t* endpts, iarray_t* accept);
vertex_t* findLeastCostPath(varray_t* feasible, int num_robs);
void findPathALL_debug(varray_t* endpts);

#endif