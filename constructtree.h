#ifndef __LABEL_H__
#define __LABEL_H__

#include "rrtstar.h"
#include "ltl.h"

varray_t* construct_tree(iarray_t* accept, vertex_t* init, int num_runs, float obst[][5][2], int numAPs, carray_t* alpha, float goal[][5][2], BA_t* B);
void writePlotFile(varray_t* endpts, iarray_t* accept, int call);
varray_t* findPath(varray_t* endpts, iarray_t* accept);

#endif