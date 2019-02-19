#ifndef __RRTSTAR_H__
#define __RRTSTAR_H__

#include "ltl.h"

typedef struct _vertex_t{ // needs to have loc, parent, cost, and buchi state
	float loc[2];
	struct _vertex_t* parent;
	float cost;
	int bstate;
} vertex_t;

typedef struct _varray_t{ // holds an array of pointers & length of array 
  vertex_t** arr;
  int len;
} varray_t;

void printPath(vertex_t* current);
void printVertex(vertex_t* current);
void printArray(varray_t* current);
vertex_t* newVertex(void);
vertex_t* copyVertex(vertex_t* old);
varray_t* newVArray(void);
void addToVArray(varray_t* result, vertex_t* element);
void removeFromVArray(varray_t* result, vertex_t* element);
void randLoc(vertex_t* result);
int collisionFreeMult(vertex_t* v, float obst[][5][2], int num_obstacles, int obstacle_size);
int collisionFree(float a[], float obst[][2], int obstacle_size);
float isLeft(float a[], float b[], float c[]);
float calcDistance(float a[], float b[]);
vertex_t* steer(vertex_t* rand, vertex_t* nearest);
float calcRadius(int count, int num_robots);
varray_t* findNearest(varray_t* endpts, vertex_t* rand);
varray_t* findNear(varray_t* endpts, vertex_t* new_steer, int count, int num_robots);
int contains(varray_t* result, vertex_t* current);
float det(float a[], float b[]);
int obstacleFree(float a[], float b[], float obst[][2], int obstacle_size);
int obstacleFreeMult(vertex_t* nearest, vertex_t* new_steer, float obst[][5][2], int num_obstacles, int obstacle_sizes);
float calcCost(vertex_t* nearest, vertex_t* new_steer);
varray_t* findMinCost(varray_t* near, vertex_t* new_steer, float obst[][5][2], int num_obstacles, int obstacle_size);
void extend(varray_t* endpts, vertex_t* copy_steer);
void rewire(varray_t* near, vertex_t* copy_steer, float obst[][5][2], int num_obstacles, int obstacle_size);
void freeVertices(vertex_t* current);
void freeArray(varray_t* current);

#endif 