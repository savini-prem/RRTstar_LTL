#ifndef __RRTSTAR_H__
#define __RRTSTAR_H__

#include "ltl.h"

typedef struct _position_t{
	float** coord; 
	int len; 
} position_t; 

typedef struct _vertex_t{ // needs to have position of each rob, parent, cost, and buchi state
	position_t* pos; 
	struct _vertex_t* parent;
	float cost;
	int bstate;
	int accept_count; 
} vertex_t;

typedef struct _varray_t{ // holds an array of pointers & length of array 
  vertex_t** arr;
  int len;
} varray_t;

typedef struct _intarray_t{
  int* arr;
  int len;
} intarray_t;
intarray_t* newIntArray(void);
void addToIntArray(intarray_t* result, int element);
int iarrayContains(int alpha_index, intarray_t* trans); // CHANGE TO ARRAY 


void printPath(vertex_t* current);
void printVertex(vertex_t* current);
void printArray(varray_t* current);
vertex_t* newVertex(int num_robs);
vertex_t* copyVertex(vertex_t* old);
varray_t* newVArray(void);
void addToVArray(varray_t* result, vertex_t* element);
void removeFromVArray(varray_t* result, vertex_t* element);
void randLoc(vertex_t* result, float*** obst, int num_obstacles, int obstacle_size,float*** environ);
int collisionFreeMult(float a[], float*** obst, int num_obstacles, int obstacle_size, float*** environ);
int collisionFree(float a[], float** obst, int obstacle_size);
float isLeft(float a[], float b[], float c[]);
float calcDistance(vertex_t* a, vertex_t* b);
vertex_t* steer(vertex_t* rand, vertex_t* nearest, int num_robots);
float calcRadius(int count, int num_robots);
vertex_t* findNearest(varray_t* endpts, vertex_t* rand);
varray_t* findNear(varray_t* endpts, vertex_t* new_steer, int count, int num_robots);
int contains(varray_t* result, vertex_t* current);
float det(float a[], float b[]);
int obstacleFree(float a[], float b[], float** obst, int obstacle_size);
int obstacleFreeMult(vertex_t* nearest, vertex_t* new_steer, float*** obst, int num_obstacles, int obstacle_size);
float calcCost(vertex_t* nearest, vertex_t* new_steer);
varray_t* findMinCost(intarray_t* accept, varray_t* near,  intarray_t* near_bstates, vertex_t* new_steer, float*** obst, int num_obstacles, int obstacle_size);
int samePos(vertex_t* a, vertex_t* b);
void extend(varray_t* endpts, vertex_t* copy_steer);
void rewire(intarray_t* accept, varray_t* near, vertex_t* copy_steer, float*** obst, int num_obstacles, int obstacle_size);
void freeVertices(vertex_t* current);
void freeArray(varray_t* current);

#endif 