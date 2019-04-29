#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <limits.h>
#include <time.h>
#include "ltl.h"
#include "rrtstar.h"
#include "constructtree.h"
#include "environ.h"

varray_t* construct_tree(int fix, iarray_t* accept, vertex_t* init, int num_runs, float*** obst, int numAPs, int num_robs, int num_regions, carray_t* alpha, float*** goal, BA_t* B, float*** environ, int num_obstacles, int obstacle_size, int region_size);
varray_t* findPath(varray_t* endpts, iarray_t* accept);
vertex_t* findLeastCostPath(varray_t* feasible, int num_robs);
void writePlotFile(varray_t* endpts, iarray_t* accept, int call);
void writeLeastCostPlotFile(vertex_t* min, int call);
void findPathALL_debug(varray_t* endpts);

int main(int argc, char ** argv){
  if(argc < 10){
    fprintf(stderr,"usage: ./main <NBA text file> <environment text file> <# of robots> <# of goal regions> <goal size> <# of obstacles> <obstacle size> <# of prefix runs> <# of suffix runs>\n");
    exit(EXIT_FAILURE);
  }

  // initialize timing 
  clock_t start_pre, start_suf, end_pre, end_suf;
  double time_pre, time_suf; 

  start_pre = clock(); 

  char * nbaFile = argv[1];
  char * envFile = argv[2]; 
  int num_robs = atoi(argv[3]);
  int num_regions = atoi(argv[4]); 
  int numAPs = num_robs * num_regions; 
  int region_size = atoi(argv[5]); 
  int num_obstacles = atoi(argv[6]);
  int obstacle_size = atoi(argv[7]);
  int num_pruns = atoi(argv[8]);
  int num_sruns = atoi(argv[9]);
  float optimal_cost; 

  //input environment
  float* loc = (float*)malloc(2*sizeof(float));
  float*** environ = initPolygon(1,4); 
  float*** obst = initPolygon(num_obstacles, obstacle_size);
  float*** goal = initPolygon(num_regions, region_size);
  getEnvironment(loc, envFile, environ, obst, num_obstacles, obstacle_size, goal, num_regions, region_size);
  vertex_t* init_pre = newVertex(num_robs);
  for(int i=0; i<init_pre->pos->len; i++){
    init_pre->pos->coord[i][0] = loc[0]; 
    init_pre->pos->coord[i][1] = loc[1]; 
  } 
  init_pre->bstate = 0;
  //printPolygon(environ,1,4);
  //printPolygon(obst, num_obstacles, obstacle_size);
  //printPolygon(goal, num_regions, region_size);
  //printVertex(init_pre); 
  
  // create state array, s 
  // ex: s = {init, accept_1}
  carray_t * s = readStates(nbaFile);
  //printStates(s);

  // create alphabet, alpha
  // ex: 2 APS => 10, 01, 11
  carray_t * alpha = createAlpha(numAPs,num_robs,num_regions);
  //printAlpha(alpha);

  // create BA
  // ex: empty 2x2 matrix (because 2 states in state array)
  BA_t * B = initBA(s);
  //printBA(B);

  // create labels, i.e. fill empty BA (each entry is an index of alphabet)
  label(B,s,alpha,nbaFile,numAPs); 
  /*iarray_t* temp = newIArray(); 
  addToIArray(temp,47); //SIMB
  addToIArray(temp,4); //SIMA
  B->trans[0][1] = temp; */
  //printBA(B);
  
  //***CONSTRUCTION OF PREFIX PLANS
  //***define goal set 

  //find accepting state indices (i.e. prefix goal states!!) 
  iarray_t* accept_pre = findAcceptStates(s);

  //***create initial state 
  // create starting vertex with random location and init buchi state
  time_t t;
  srand((unsigned) time(&t));

  // construct tree 
  varray_t* endpts_pre = construct_tree(0, accept_pre, init_pre, num_pruns, obst, numAPs, num_robs, num_regions, alpha, goal, B, environ, num_obstacles, obstacle_size, region_size);

  //for every member of P find path
  //print all paths that reach goal region & add to array
  printf("All Feasible Prefix Paths:\n");
  varray_t* feasible_pre = findPath(endpts_pre, accept_pre);
  vertex_t* min_pre = findLeastCostPath(feasible_pre, num_robs);
  writeLeastCostPlotFile(min_pre, 0);
  
  end_pre = clock();  
  time_pre = ((double) (end_pre - start_pre)) / CLOCKS_PER_SEC;

// CONSTRUCTION OF SUFFIX PLANS 
  start_suf = clock();

//for every member of P 
  for(int i=0; i<feasible_pre->len; i++){
// create initial state 
    vertex_t* init_suf = copyVertex(min_pre->parent);
    init_suf->bstate = min_pre->bstate;
    init_suf->parent = NULL;
    init_suf->cost = 0;

//define goal set
  	iarray_t* accept_suf = newIArray();
  	addToIArray(accept_suf, init_suf->bstate); //not efficient cause repeats

// construct tree
    varray_t* endpts_suf = construct_tree(1, accept_suf, init_suf, num_sruns, obst, numAPs, num_robs, num_regions, alpha, goal, B, environ, num_obstacles, obstacle_size, region_size);

// for every member of S find path
	  if(feasible_pre->arr[i]==min_pre) {
      printf("All Feasible Suffix Paths for Least-Cost Prefix Path:\n");
      varray_t* feasible_suf = findPath(endpts_suf, accept_suf);
      vertex_t* min_suf = findLeastCostPath(feasible_suf, num_robs);
		  writeLeastCostPlotFile(min_suf, 1);
		  optimal_cost = 0.5*(min_pre->cost + min_suf->cost); 
	  }
  }

// CONSTRUCTION OF OPTIMAL PLANS
  end_suf = clock(); 
  time_suf = ((double) (end_suf - start_suf)) / CLOCKS_PER_SEC;

  printf("Prefix Runtime: %f\n", time_pre);
  printf("Suffix Runtime: %f\n", time_suf);
  printf("Number of Accepting States: %d\n", feasible_pre->len); 
  printf("Cost of Optimal Plan: %f\n", optimal_cost);

  return EXIT_SUCCESS;
}