//***convert to nba

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <limits.h>
#include <sys/time.h>
#include "ltl.h"
#include "rrtstar.h"
#include "constructtree.h"

#define NUM_ROBOTS 1
#define NUM_OBSTACLES 2
#define OBSTACLE_SIZE 4
#define NUM_GOALS 1
#define GOAL_SIZE 3

varray_t* construct_tree(iarray_t* accept, vertex_t* init, int num_runs, float obst[][5][2], int numAPs, carray_t* alpha, float goal[][5][2], BA_t* B);
varray_t* findPath(varray_t* endpts, iarray_t* accept);
void writePlotFile(varray_t* endpts, iarray_t* accept, int call);

int main(int argc, char ** argv){
  /*
  if(argc < 9){
    fprintf(stderr,"usage: ./main1 <Obstacle Text File> <Goal Text File> 
    <NBA Text File> <AP Mapping Text File> <nMaxPre> <nMaxSuf> 
    <#APS> <#Bots> <#Locations> \n");
    exit(EXIT_FAILURE);
  }*/

  if(argc < 5){
    fprintf(stderr,"usage: ./main1 <NBA Text File> <# of APs> <# prefix runs> <# suffix runs>\n");
    exit(EXIT_FAILURE);
  }

  // hardcoded for now
  // obstacles: n+1 vertices (listed on counterclockwise order)
  float obst[2][5][2] = {{{0.3, 0.2}, {0.3, 0.0}, {0.7, 0.0}, {0.7, 0.2}, {0.3, 0.2}},
                         {{0.4, 1.0}, {0.4, 0.7}, {0.6, 0.7}, {0.6, 1.0}, {0.4, 1.0}}};
  // goals: n+1 vertices (listed on counterclockwise order)
  float goal[2][5][2] = {{{0.3, 0.5}, {0.3, 0.3}, {0.5, 0.3}, {0.3, 0.5}, {0.3, 0.3}},
  						 {{0.1, 0.9}, {0.1, 0.7}, {0.3, 0.7}, {0.1, 0.9}, {0.1, 0.7}}}; 

  char * nbaFile = argv[1];
  int numAPs = atoi(argv[2]);
  int num_pruns = atoi(argv[3]);
  int num_sruns = atoi(argv[4]);
  
  // create state array, s 
  // ex: s = {init, accept_1}
  carray_t * s = readStates(nbaFile);
  //printStates(s);

  // create alphabet, alpha
  // ex: 2 APS => 10, 01, 11
  carray_t * alpha = createAlpha(numAPs);
  //printAlpha(alpha);

  // create BA
  // ex: empty 2x2 matrix (because 2 states in state array)
  BA_t * B = initBA(s);

  // create labels, i.e. fill empty BA (each entry is an index of alphabet)
  label(B,s,alpha,nbaFile,numAPs); 
  //printBA(B);

  //***CONSTRUCTION OF PREFIX PLANS
  //***define goal set 

  //find accepting state indices (i.e. prefix goal states!!) 
  iarray_t*  accept_pre = findAcceptStates(s);

  //***create initial state 
  // create starting vertex with random location and init buchi state
  time_t t; 
  srand((unsigned) time(&t));
  vertex_t* init_pre = newVertex();
  while(1){
    randLoc(init_pre); 
    if(collisionFreeMult(init_pre,obst,NUM_OBSTACLES,OBSTACLE_SIZE)) break;
  }
  init_pre->bstate = 0;

  // construct tree 
  varray_t* endpts_pre = construct_tree(accept_pre, init_pre, num_pruns, obst, numAPs, alpha, goal, B);

  //for every member of P find path
  //print all paths that reach goal region & add to array
  printf("All Feasible Prefix Paths:\n");
  varray_t* feasible_pre = findPath(endpts_pre, accept_pre);
  writePlotFile(endpts_pre, accept_pre, 0);


// CONSTRUCTION OF SUFFIX PLANS 
//for every member of P 
  for(int i=0; i<feasible_pre->len; i++){

// create initial state 
// rip lol my alg slightly diff (im just making a copy this isnt true)
  	vertex_t* init_suf = copyVertex(feasible_pre->arr[i]);
  	init_suf->parent = NULL;

//define goal set
  	//iarray_t* accept_suf = newIArray();
  	//addToIArray(accept_suf, init_suf->bstate); //not efficient cause repeats
  	iarray_t* accept_suf = getSuffixGoals(B, init_suf->bstate);

// construct tree
    varray_t* endpts_suf = construct_tree(accept_suf, init_suf, num_sruns, obst, numAPs, alpha, goal, B);

// for every member of S find path
    printf("All Feasible Suffix Paths for Prefix Path %d:\n", i);
    varray_t* feasible_suf = findPath(endpts_suf, accept_suf);
    writePlotFile(endpts_suf, accept_suf, i+1);
  }

// CONSTRUCTION OF OPTIMAL PLANS
  
  return EXIT_SUCCESS;
}