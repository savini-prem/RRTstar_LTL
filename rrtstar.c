#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "ltl.h"
#include "rrtstar.h"

#define TRUE 1
#define FALSE 0

int iarrayContains(int alpha_index, intarray_t* trans){ // CHANGE TO ARRAY 
  for(int i=0; i<trans->len; i++){
    if(trans->arr[i] == alpha_index) return TRUE;
  }
  return FALSE;
}

intarray_t* newIntArray(void){
  intarray_t* result = (intarray_t*)malloc(sizeof(intarray_t));
    result->arr = NULL;
    result->len = 0; 
    return result;
}

void addToIntArray(intarray_t* result, int element){
  result->arr = (int*)realloc(result->arr,(result->len+1)*sizeof(int));
  result->arr[result->len] = element;
  result->len++;
}

// print current tree
void printPath(vertex_t* current){
	printVertex(current);
	while(current->parent!=NULL){
		current = current->parent;
		printVertex(current);
	}
}

// for debugging 
void printVertex(vertex_t* current){
	printf("[");
	for(int i=0; i<current->pos->len; i++){
		printf("(%f, %f), ",current->pos->coord[i][0],current->pos->coord[i][1]); 
	}
	printf("%d]\n",current->bstate);
}

// for debugging
void printArray(varray_t* current){
	for(int i=0; i<current->len; i++){
		printVertex(current->arr[i]);
	}
    printf("\n");
}

// create new vertex and initialize 
vertex_t* newVertex(int num_robs){
	vertex_t* result = (vertex_t*)malloc(sizeof(*result));
	result->pos = (position_t*)malloc(sizeof(*result->pos));
	result->pos->coord = malloc(num_robs*sizeof(*result->pos->coord));
	for(int i=0; i<num_robs; i++){
		result->pos->coord[i] = malloc(2*sizeof(*result->pos->coord[i])); 
		result->pos->coord[i][0] = 0.0;
		result->pos->coord[i][1] = 0.0;
	}
	result->pos->len = num_robs; 
	result->parent=NULL;
	result->cost=0.0;
	result->bstate=-1;
	result->accept_count = 0; 
	return result;
}

// create new vertex and initialize with contents of old vertex
vertex_t* copyVertex(vertex_t* old){
	vertex_t* result = (vertex_t*)malloc(sizeof(vertex_t));
	result->pos = (position_t*)malloc(sizeof(*result->pos));
	result->pos->coord = malloc(old->pos->len*sizeof(*result->pos->coord));
	for(int i=0; i<old->pos->len; i++){
		result->pos->coord[i] = malloc(2*sizeof(*result->pos->coord[i])); 
		result->pos->coord[i][0] = old->pos->coord[i][0];
		result->pos->coord[i][1] = old->pos->coord[i][1];
	}
	result->pos->len = old->pos->len; 
	result->parent=old->parent;
	result->cost=old->cost;
	result->bstate=old->bstate;
	result->accept_count = old->accept_count;  
	return result;
}

// create new vertex array and initialize
varray_t* newVArray(void){
	varray_t* result = (varray_t*)malloc(sizeof(varray_t));
  	result->arr = NULL;
  	result->len = 0; 
  	return result;
}

// add new vertex element to end of array 
void addToVArray(varray_t* result, vertex_t* element){
	result->arr = (vertex_t**)realloc(result->arr,(result->len+1)*sizeof(vertex_t*));
    result->arr[result->len] = element;
    result->len++;
}

void removeFromVArray(varray_t* result, vertex_t* element){ //NOTE THAT MALLOC-ED SIZE WILL NOT GET SMALLER
	for(int i=0; i<result->len; i++){
		if(result->arr[i]==element){
			if(i==result->len-1) result->arr[i] = NULL;
			for(int j=i; j<result->len-1; j++){
				result->arr[j] = result->arr[j+1];
			}
			break;
		}
	}
	result->len--;
}

// choose random location and assign to vertex (NOTE: gives random coords for ALL robots)
void randLoc(vertex_t* result, float*** obst, int num_obstacles, int obstacle_size,float*** environ){ //add obstacle check to this
	float loc[2]; 
	int maxX = environ[0][0][0], maxY = environ[0][0][1], minX = environ[0][0][0], minY = environ[0][0][1]; 
	for(int i=0; i<4; i++){
		if(environ[0][i][0]>maxX) maxX = environ[0][i][0];
		if(environ[0][i][1]>maxY) maxY = environ[0][i][1];
		if(environ[0][i][0]<minX) minX = environ[0][i][0];
		if(environ[0][i][1]<minY) minY = environ[0][i][1];
	}
	for(int i=0; i<result->pos->len; i++){
		while(1){
			loc[0] = ((float) rand()) / ((float) RAND_MAX) * (maxX-minX);
			loc[1] = ((float) rand()) / ((float) RAND_MAX) * (maxY-minY);
			if(collisionFreeMult(loc,obst,num_obstacles,obstacle_size,environ)) {
				result->pos->coord[i][0] = loc[0];
				result->pos->coord[i][1] = loc[1]; 
				break;
			}
		}
	}
}

// returns TRUE if location does not collide with ALL obstacles OR IS IN BOUNDS OF ENV
int collisionFreeMult(float a[], float*** obst, int num_obstacles, int obstacle_size, float*** environ){
	int result = TRUE;
	for(int i=0; i<num_obstacles; i++){
		if(!collisionFree(a,obst[i],obstacle_size)){
			result = FALSE;
			break;
		}

		if(collisionFree(a,environ[0],4)){
			result = FALSE;
			break;
		}
	}
	return result;
}

// returns TRUE if vertex does not collide with one obstacle
int collisionFree(float a[], float** obst, int obstacle_size){
	// pseudo-code from: http://geomalgorithms.com/a03-_inclusion.html#cn_PnPoly()
	int counter = 0;

	for(int i=0; i<obstacle_size; i++){
		//if(isLeft(obst[i],obst[i+1],a)==0) return 0;
		if(obst[i][1]<=a[1] && obst[i+1][1]>a[1]){
			if(isLeft(obst[i],obst[i+1],a)>0) ++counter;
		}
		if(obst[i][1]>a[1] && obst[i+1][1]<=a[1]){
			if(isLeft(obst[i],obst[i+1],a)<0) --counter;
		}
	}
	//return !counter;
	if(counter == 0) return 1; 
	else return 0; 
}

// for collisionFree
float isLeft(float a[], float b[], float c[]){
	return ( (b[0]-a[0])*(c[1]-a[1]) - (c[0]-a[0])*(b[1]-a[1]) );
}


float calcDistance(vertex_t* a, vertex_t* b){
	float result = 0; 
	for(int i=0; i<a->pos->len; i++){
		result += pow(a->pos->coord[i][0] - b->pos->coord[i][0],2) + pow(a->pos->coord[i][1] - b->pos->coord[i][1],2);
	}
	result = sqrt(result);
	return result; 
}

// changes location of rand vertex 
// new location ensures distance between nearest and new rand < nu
// new location ensures distance between old rand and new rand < old rand and nearest
// i.e that new rand lies on the line between old rand and nearest, a distance of nu away from nearest
vertex_t* steer(vertex_t* rand, vertex_t* nearest, int num_robots){ 
	// calculate nu
	float nu = 0.50*num_robots; //because environment is 1x1

	for(int i=0; i<rand->pos->len; i++){
		float u[] = {rand->pos->coord[i][0],rand->pos->coord[i][1]};
		float v[] = {nearest->pos->coord[i][0],nearest->pos->coord[i][1]};

		// calculate direction vector 
		float dv[] = {u[0]-v[0],u[1]-v[1]};
	
		// calculate distance between nearest & rand
		//float a = calcDistance(nearest->pos->coord[i],rand->pos->coord[i]);
		float a = sqrt( pow(v[0]-u[0],2) + pow(v[1]-u[1],2) );

		// scale direction vector by nu
		float w[] = {(nu/a)*dv[0],(nu/a)*dv[1]};

		// find new rand (only if nu/a < 1)
		if(nu/a < 1){
			rand->pos->coord[i][0] = v[0]+w[0];
			rand->pos->coord[i][1] = v[1]+w[1];
		}	
	}
	return rand;
}

// calculates radius used by near 
// CONFIRM WITH XUSHENG THAT dim = 2*NUM_ROBOTS AND THAT mu = 1
float calcRadius(int count, int num_robots){ 
	float nu = 0.25*num_robots; //because environment is 1x1
	float gamma = ceil(4*pow((1/M_PI),1/(2*num_robots)));
	float radius = fmin( ( gamma*pow( log(count)/count , 1/(2*num_robots) ) ), nu );
	return radius;
	//return 10; 
}

vertex_t* findNearest(varray_t* endpts, vertex_t* rand){ // not efficient implementation because repeat nodes	
	vertex_t* result=endpts->arr[0];
	int dist = calcDistance(endpts->arr[0],rand);
									
	for(int i=0; i<endpts->len; i++){
		vertex_t* current = endpts->arr[i];

		int dist1 = calcDistance(current,rand);
		if(dist1<dist) result = current;
		
		while(current->parent!=NULL){
			current = current->parent;
			int dist1 = calcDistance(current,rand);
			if(dist1<dist){
				dist = dist1;
				result = current;
			}
		}
	}
	return result;
}

// returns ALL vertices in tree that are within a max radial distance from new_steer 
varray_t* findNear(varray_t* endpts, vertex_t* new_steer, int count, int num_robots){
	varray_t* result = newVArray();

	for(int i=0; i<endpts->len; i++){
		vertex_t* current = endpts->arr[i]; 
 
		float dist = calcDistance(current,new_steer);
		float radius = calcRadius(count, num_robots);
		//printf("dist,radius: %f,%f\n",dist,radius);

		if(dist<radius && !contains(result,current)){
			addToVArray(result,current);
		}

		while(current->parent!=NULL){
			current = current->parent;
			dist = calcDistance(current,new_steer);
			//printf("dist,radius: %f,%f\n",dist,radius);
			if(dist<radius && !contains(result,current)){
				addToVArray(result,current);
			}
		}
	}

	return result;
}

// returns true if vertex array contains current
int contains(varray_t* result, vertex_t* current){
	for(int i=0; i<result->len; i++){
		if(result->arr[i]==current) return TRUE;
	}
	return FALSE;
}


// for obstacle free, calculates determinant
float det(float a[], float b[]){
	float result = a[0]*b[1] - a[1]*b[0];
	return result; 
}

// returns TRUE if the line between two vertices doesn't intersect one obstacle
int obstacleFree(float a[], float b[], float** obst, int obstacle_size){
	// pseudo-code from: http://geomalgorithms.com/a13-_intersect-4.html
	float tE = 0.0;
	float tL = 1.0;
	float t, N, D;
	float dS[] = {b[0]-a[0], b[1]-a[1]};

	for(int i = 0; i<obstacle_size; i++){
		float edge[] = {obst[i+1][0]-obst[i][0], obst[i+1][1]-obst[i][1]};
		float intermediate[] = {a[0]-obst[i][0], a[1]-obst[i][1]};
		N = det(edge,intermediate);
		D = -det(edge,dS);
		t = N/D;
		
		if(fabs(D)==0.0){
			if(N<0) return TRUE;
			else continue;
		}

		if(D<0){
			if(t>tE){
				tE = t;
				if(tE>tL) return TRUE;
			}
		}

		if(D>0){
			if(t<tL){
				tL = t;
				if(tL<tE)return TRUE;
			}
		}
	}
	return FALSE;
}

// returns TRUE if the line between two vertices doesn't intersect ALL obstacles
int obstacleFreeMult(vertex_t* nearest, vertex_t* new_steer, float*** obst, int num_obstacles, int obstacle_size){
	for(int k=0; k<nearest->pos->len; k++){
		
		// start and end points of line segment 
		float a[] = {nearest->pos->coord[k][0],nearest->pos->coord[k][1]};
		float b[] = {new_steer->pos->coord[k][0],new_steer->pos->coord[k][1]};

		for(int i=0; i<num_obstacles; i++){
			if(!obstacleFree(a,b,obst[i],obstacle_size)){
				return FALSE; 
			}
		}

	}
	return TRUE;
}

// calculates cost of adding vertex (note that C=1)
float calcCost(vertex_t* nearest, vertex_t* new_steer){
	return calcDistance(nearest,new_steer) + nearest->cost;
}

varray_t* findMinCost(intarray_t* accept, varray_t* near, intarray_t* near_bstates, vertex_t* new_steer, float*** obst, int num_obstacles, int obstacle_size){
	varray_t* result = newVArray();

	for(int j=0; j<near_bstates->len; j++){
		vertex_t* min;
		for(int i=0; i<near->len; i++){
			if(!iarrayContains(near->arr[i]->bstate,accept)){
				if(obstacleFreeMult(near->arr[i], new_steer, obst, num_obstacles, obstacle_size)){
					if(near->arr[i]->bstate==near_bstates->arr[j]) min = near->arr[i];
				}
			}
		}

		int flag = 0;
		for(int i=0; i<near->len; i++){
			if(near->arr[i]->bstate==near_bstates->arr[j]) {
				if(!iarrayContains(near->arr[i]->bstate,accept)){
					// check if no obstacles in path between near and new_steer 
					if(obstacleFreeMult(near->arr[i], new_steer, obst, num_obstacles, obstacle_size)){
						if(calcCost(near->arr[i],new_steer) < calcCost(min,new_steer)) min = near->arr[i];
						flag++;
					}
				}
			}
		}

		if(flag != 0) {
			for(int i=0; i<near->len; i++){
				if(!iarrayContains(near->arr[i]->bstate,accept)){
					if(samePos(min,near->arr[i])==1) addToVArray(result,near->arr[i]);
				}
			}
		}
	}

	return result;
}

int samePos(vertex_t* a, vertex_t* b){
	int result = 0; 
	int count = 0;  
	for(int i=0; i<a->pos->len; i++){
		if(a->pos->coord[i][0]==b->pos->coord[i][0] && a->pos->coord[i][1]==a->pos->coord[i][1]) count++;
	}
	if(count == a->pos->len) result = 1; 
	return result; 
}

void extend(varray_t* endpts, vertex_t* copy_steer){	
	// update endpts array
	int flag = 0; 
    for(int i=0; i<endpts->len; i++){
       	if(endpts->arr[i]==copy_steer->parent){
          	flag++;
        }
    }
    if(flag!=0) removeFromVArray(endpts,copy_steer->parent); //doesnt branch
}

void rewire(intarray_t* accept, varray_t* near, vertex_t* copy_steer, float*** obst, int num_obstacles, int obstacle_size){
	for(int i=0; i<near->len; i++){ 
		if(!iarrayContains(near->arr[i]->bstate,accept)){
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

// free heap memory
void freeVertices(vertex_t* current){
	vertex_t* parentVertex = current->parent;
	free(current);
	while(parentVertex!=NULL){
		current = parentVertex;//maybe problem
		parentVertex = parentVertex->parent;
		free(current);
	}
}

// free heap memory
void freeArray(varray_t* current){
	for(int i=0; i<current->len; i++){
		free(current->arr[i]);
	}
	free(current);
}