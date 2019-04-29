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

  return feasible;
}

vertex_t* findLeastCostPath(varray_t* feasible, int num_robs){
  vertex_t* min = newVertex(num_robs);
  if(feasible->len !=0){
    printf("Least-Cost Path:\n");

    min = feasible->arr[0];
    for(int i=0; i<feasible->len; i++){
      if(feasible->arr[i]->cost<min->cost) min = feasible->arr[i];
    }
    printPath(min);
  }
  else printf("None\n");

  printf("\n");
  return min; 
}

void writeLeastCostPlotFile(vertex_t* min, int call){
    vertex_t* start = min;
    vertex_t* current = min; 

    for(int j=0; j<min->pos->len; j++){
      char filename[100];
      sprintf(filename, "path%d-%d.txt", j, call);
      FILE *f = fopen(filename, "w");

      for(int i=0; i<2; i++){
        fprintf(f, "%f ", current->pos->coord[j][i]);
        while(current->parent!=NULL){
          current = current->parent;
          fprintf(f, "%f ", current->pos->coord[j][i]);
        }
        fprintf(f, "\n");
        current = start;
      }

      fclose(f);
    }

}

varray_t* construct_tree(int fix, iarray_t* accept, vertex_t* init, int num_runs, float*** obst, int numAPs, int num_robs, int num_regions, carray_t* alpha, float*** goal, BA_t* B, float*** environ, int num_obstacles, int obstacle_size, int region_size){
  // BEGIN TREE CONSTRUCTION
  // initialize endpoint array
  varray_t* endpts = newVArray();

  // start count of vertices in tree
  int count = 0;

  // increment accept_count 
  if(arrayContains(init->bstate,accept)) init->accept_count++; 

  // add init to endpoint array & increase count
  addToVArray(endpts, init);
  count++;

  int flag = 0;
  // iterate num_runs times or until path is found 
  for(int k=0; k<num_runs; k++){
    //vertex_t* rand = sample_pts->arr[k]; 
    // create new vertex with random locations of each rob
    vertex_t* rand = newVertex(num_robs); 
    randLoc(rand, obst, num_obstacles, obstacle_size,environ); 

    // initialize pointer to the nearest node 
    vertex_t* nearest = findNearest(endpts,rand);
    //printf("nearest: ");
    //printVertex(nearest);

    // create (actually replace rand) new node using steer
    /*vertex_t* new_steer = steer(rand, nearest, num_robs);
    int marker = 0; 
    for(int n=0; n<new_steer->pos->len; n++){
      if(collisionFreeMult(new_steer->pos->coord[n],obst,num_obstacles,obstacle_size,environ)==0) marker = 1; 
    }
    if(marker == 1) continue;*/ 
    vertex_t* new_steer = rand; 
    //printf("RAND!!!: ");
    //printVertex(rand);

    // find ALL vertices in tree that are within a max radial distance from new_steer 
    varray_t* near = findNear(endpts,new_steer,count,num_robs);
    if(near->len == 0) continue;
    //printArray(near);

    //EXTEND 
    for(int s=0; s<B->DIM; s++){
      //printf("dim: %d", s);
      vertex_t* copy_steer = copyVertex(new_steer);
      copy_steer->bstate = s; 
      //printf("copy_steer: "); 
      //printVertex(copy_steer);

      vertex_t* min = copy_steer;
      for(int x=0; x<near->len; x++){
        int trueAP = whichAPIsTrue(alpha, numAPs, near->arr[x], goal, num_regions, region_size);
        //printf("trueAP: %d\n", trueAP);
        iarray_t* next_states = nextBState(near->arr[x]->bstate, trueAP, B); 
        //for(int g=0; g<next_states->len; g++) printf("%d ",next_states->arr[g]);
        //printf("\n");
        if(arrayContains(s,next_states)){ 
          //if(!arrayContains(near->arr[x]->bstate,accept)){
          if(!fix && near->arr[x]->accept_count<1){
            if(obstacleFreeMult(near->arr[x], copy_steer, obst, num_obstacles, obstacle_size)){
              min = near->arr[x];
            }
          }
          if(fix && near->arr[x]->accept_count<2){
            if(obstacleFreeMult(near->arr[x], copy_steer, obst, num_obstacles, obstacle_size)){
              min = near->arr[x];
            }
          }
        }
      }

      if(min == copy_steer) continue; 

      for(int x=0; x<near->len; x++){   
        int trueAP = whichAPIsTrue(alpha, numAPs, near->arr[x], goal, num_regions, region_size);
        iarray_t* next_states = nextBState(near->arr[x]->bstate, trueAP, B); 
        if(arrayContains(s,next_states)){     
          //if(!arrayContains(near->arr[x]->bstate,accept)){
          if(!fix && near->arr[x]->accept_count<1){
            if(obstacleFreeMult(near->arr[x], copy_steer, obst, num_obstacles, obstacle_size)){
              if(calcCost(near->arr[x],copy_steer) < calcCost(min,copy_steer)) min = near->arr[x];
            }
          }
          if(fix && near->arr[x]->accept_count<2){
            if(obstacleFreeMult(near->arr[x], copy_steer, obst, num_obstacles, obstacle_size)){
              if(calcCost(near->arr[x],copy_steer) < calcCost(min,copy_steer)) min = near->arr[x];
            }
          }
        }
      }

      copy_steer->parent = min; 
      copy_steer->cost = calcCost(min,copy_steer);
      //printf("min: ");
      //printVertex(min);

      addToVArray(endpts,copy_steer);
      extend(endpts,copy_steer);//check what this does 
      count++;
      if(arrayContains(copy_steer->bstate,accept)) copy_steer->accept_count = min->accept_count + 1;
      else copy_steer->accept_count = min->accept_count; 


      //REWIRE
      int trueAP = whichAPIsTrue(alpha, numAPs, copy_steer, goal, num_regions, region_size);
      iarray_t* new_next_states = nextBState(copy_steer->bstate, trueAP, B); 
      for(int i=0; i<near->len; i++){ 
        if(isConnectionValid(near->arr[i]->bstate, new_next_states)){
          if(!arrayContains(copy_steer->bstate,accept)){
            // check if no obstacles in path between near and copy_steer
            if(obstacleFreeMult(near->arr[i], copy_steer, obst, num_obstacles, obstacle_size)){
              // check if new path cheaper than original
              if(calcCost(copy_steer,near->arr[i]) < near->arr[i]->cost){
                near->arr[i]->parent = copy_steer;
                near->arr[i]->cost = calcCost(copy_steer,near->arr[i]);
              }
            }
          }
        }
      }

      //if suffix then close loop
      if(fix && copy_steer->accept_count==2){
        if(obstacleFreeMult(init, copy_steer, obst, num_obstacles, obstacle_size)){
          vertex_t* end = copyVertex(init);
          end->parent=copy_steer;
          end->cost=calcCost(copy_steer,end);
          end->bstate=copy_steer->bstate;
          end->accept_count = copy_steer->accept_count+1; 

          addToVArray(endpts,end);
          //extend(endpts,end);
          removeFromVArray(endpts,end->parent);
        }
      }

    }

  }
  return endpts;
} 