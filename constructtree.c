#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <limits.h>
#include <sys/time.h>
#include "ltl.h"
#include "rrtstar.h"

#define NUM_ROBOTS 1
#define NUM_OBSTACLES 2
#define OBSTACLE_SIZE 4
#define NUM_GOALS 1
#define GOAL_SIZE 3

/* TO DO: 
  - fix checking how APs are tru cause currently assuming 1 robot 1 goal 
  - check: #APS = #robots * #goals
*/
varray_t* findPath(varray_t* endpts, iarray_t* accept){
  varray_t* feasible = newVArray();
  for(int i=0; i<endpts->len; i++){
    for(int j=0; j<accept->len; j++){
      if(endpts->arr[i]->bstate == accept->arr[j]){
        printf("Path %d:\n",feasible->len);
        printPath(endpts->arr[i]);
  
        addToVArray(feasible,endpts->arr[i]);
      }
    }
  }

  //print least-cost path of paths that reach goal region
  //vertex_t* result = findMinCost(feasible, ??, obst, NUM_OBSTACLES, OBSTACLE_SIZE);
  //varray_t* findMinCost(varray_t* near, vertex_t* new_steer, float obst[][5][2], int num_obstacles, int obstacle_size){
  //FUCK NEED NEW FIND MIN COST
  if(feasible->len !=0){
    printf("Least-Cost Path:\n");

    vertex_t* min = feasible->arr[0]; 
    for(int i=0; i<feasible->len; i++){
      if(feasible->arr[i]->cost<min->cost) min = feasible->arr[i];
    }
    printPath(min);
  }
  else printf("None\n");

  printf("\n");
  return feasible;
}

void writePlotFile(varray_t* endpts, iarray_t* accept, int call){
    char filename[] = "path_.txt";
    filename[4] = call+'0';
    FILE *f = fopen(filename, "w");

    for(int i=0; i<endpts->len; i++){
      for(int j=0; j<accept->len; j++){
        if(endpts->arr[i]->bstate == accept->arr[j]){
          vertex_t* current = endpts->arr[i];
          vertex_t* start = current;
          for(int i=0; i<2; i++){
            fprintf(f, "%f ", current->loc[i]);
            while(current->parent!=NULL){
              current = current->parent;
              fprintf(f, "%f ", current->loc[i]);
            }
            fprintf(f, "\n");
            current = start;
          }

          fprintf(f, "%d ",current->bstate);
          while(current->parent!=NULL){
            current = current->parent;
            fprintf(f, "%d ",current->bstate);
          }
          fprintf(f, "\n");
        }
      }
    }
    fclose(f);
}

varray_t* construct_tree(iarray_t* accept, vertex_t* init, int num_runs, float obst[][5][2], int numAPs, carray_t* alpha, float goal[][5][2], BA_t* B){
  // BEGIN TREE CONSTRUCTION
  // initialize endpoint array
  varray_t* endpts = newVArray();

  // start count of vertices in tree
  int count = 0;

  // add init to endpoint array & increase count
  addToVArray(endpts, init);
  count++;

  int flag = 0;
  // iterate num_runs times or until path is found 
  for(int i=0; i<num_runs; i++){
    // create new vertex with random location
    vertex_t* rand = newVertex(); 
    while(1){
      randLoc(rand); 
      if(collisionFreeMult(rand,obst,NUM_OBSTACLES,OBSTACLE_SIZE)) break;
    }

    // initialize pointer to the nearest node 
    varray_t* nearest = findNearest(endpts,rand);
    //printf("run %d:\n",i);
    //printArray(nearest);

    // create (actually replace rand) new node using steer
    vertex_t* new_steer = steer(rand, nearest->arr[0]);

    // find ALL vertices in tree that are within a max radial distance from new_steer 
    varray_t* near = findNear(endpts,new_steer,count,NUM_ROBOTS);
    if(near->len == 0) continue; 
    //printArray(near);

    // find vertices in near that yields min cost path 
    varray_t* min = findMinCost(near, new_steer, obst, NUM_OBSTACLES, OBSTACLE_SIZE);
    if(min->len == 0) continue; //new_steer to min isnt obstacle free

    // determine which APs are true 
    int trueAP = whichAPIsTrue(alpha, numAPs, new_steer->loc, goal, NUM_GOALS, GOAL_SIZE);

    for(int i=0; i<min->len; i++){
      // find next states for this min
      iarray_t* next_states = nextBState(min->arr[i]->bstate, trueAP, B); 

      if(next_states->len == 0){
        vertex_t* copy_steer = copyVertex(new_steer);
        copy_steer->parent = min->arr[i]; 
        copy_steer->cost = calcCost(min->arr[i], copy_steer);
        copy_steer->bstate = min->arr[i]->bstate;

        // add to endpt array
        addToVArray(endpts,copy_steer);

        extend(endpts,copy_steer); 

        // increment counter
        count++;
      }
      else{
        for(int j=0; j<next_states->len; j++){
          // make copy of new_steer, update attributes, and connect to min
          vertex_t* copy_steer = copyVertex(new_steer);
          copy_steer->parent = min->arr[i]; 
          copy_steer->cost = calcCost(min->arr[i], copy_steer);
          copy_steer->bstate = next_states->arr[j];

          // add to endpt array
          addToVArray(endpts,copy_steer);

          extend(endpts,copy_steer); 
          // increment counter
          count = count + next_states->len;
        
          iarray_t* new_next_states = nextBState(copy_steer->bstate, trueAP, B); 
          if(isConnectionValid(near->arr[i]->bstate, new_next_states)){
            rewire(near,copy_steer,obst, NUM_OBSTACLES, OBSTACLE_SIZE); 
          }  

          //for(int i=0; i<accept->len; i++){ //maybe move this because not all copy steers are being used
           // if(accept->arr[i] == copy_steer->bstate) break;
          //}
          //if(accept->arr[0] == copy_steer->bstate) break;
        }
      }

      for(int j=0; j<accept->len; j++){
        for(int i=0; i<next_states->len; i++){
          if(next_states->arr[i]==accept->arr[i]) flag = 1; //CHANGE THIS TO A LOOP OF ALL ACCEPTING STATES 
                                                          //(except make sure all accepting states can be explored!)
        }                                                   // maybe use continue! statement
      }                                                //this is how to get multiple feasible paths! 
    }
      
    // break if vertex in goal region ALSO CHANGE THIS (maybe change to if trueAP=finalstate thenbreak)
    //if(!collisionFreeMult(new_steer,goal,NUM_GOALS,GOAL_SIZE)) break;
    //if(trueAP==1) break;
    if(!(flag==0)) break; //change to continue, except dont allow this branch to be extended :/

    // free allocated memory in near
    //freeArray(near);    
  }
  
  // print all paths
  /*printf("\nAll Paths:\n");
  for(int i=0; i<endpts->len; i++){
    printf("Path %d:\n",i);
    printPath(endpts->arr[i]);
  }
  printf("\n");*/

  // print least-cost path of paths that reach goal region
  /*printf("Least-Cost Path:\n");
  vertex_t* result = findMinCost(feasible);
  printPath(result);
  printf("\n");*/

  // write to plot file (change this to looping thru accept array!)
  //writePlotFile(endpts,accept);

  // free all allocated memory (one day run in valgrind to see if works lol)
  /*for(int i=0; i<endpts->len; i++){ 
    freeVertices(endpts->arr[i]);
  }
  freeArray(endpts);
  freeArray(feasible);*/
  return endpts;
} 