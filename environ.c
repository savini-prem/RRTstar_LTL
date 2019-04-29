#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "environ.h"

#define LINE_SIZE 100

float*** initPolygon(int num_polygons, int num_vertices){
	float*** shape = (float***)malloc(num_polygons * sizeof(shape));
	for(int i=0; i<num_polygons; i++){
		shape[i] = (float**)malloc((num_vertices+1)* sizeof(shape[i]));
	}

	for(int i=0; i<num_polygons; i++){
		for(int j=0; j<num_vertices+1; j++){
			shape[i][j] = (float*)malloc(sizeof(shape[i][j]));
		}
	}

	for(int i=0; i<num_polygons; i++){
		for(int j=0; j<num_vertices+1; j++){
			for(int k=0; k<2; k++){
				shape[i][j][k] = *(float*)malloc(sizeof(shape[i][j][k]));
				shape[i][j][k] = 0; 
			}
		}
	}
	return shape;
}

void fillPolygon(char* line, float** shape){
	printf("line: %s", line);
    	
    const char* pch = strtok (line," ");
   	printf ("%s\n",pch);

    sscanf(pch,"(%f,%f)",&shape[0][0],&shape[0][1]);

    int j=1;
  	while (pch != NULL){
    	pch = strtok(NULL, " ");

    	printf ("%s\n",pch);

     	j++;
  	}
  	shape[j][0] = shape[0][0];
    shape[j][1] = shape[0][1];
}

void printPolygon(float*** shape, int num_polygons, int num_vertices){
	for(int i=0; i<num_polygons; i++){
		printf("Shape %d:\n", i);
		for(int j=0; j<num_vertices+1; j++){
			float pos1 = shape[i][j][0];
			float pos2 = shape[i][j][1];
			printf("(%f,%f)\n",pos1,pos2);
		}
	}
	printf("\n");
}

void getEnvironment(float* loc, char* filename, float*** environ, float*** obst, int num_obstacles, int obstacle_size, float*** goal, int num_regions, int region_size){
	FILE * f = fopen(filename, "r");
  	char* line = (char*)malloc(LINE_SIZE*sizeof(char));

  	// init polygon array data structure 
  	int count = 0; 
  	rewind(f);

  	while(fgets(line, LINE_SIZE, f) != NULL){
  		if(count<num_obstacles){
    		const char* pch = strtok(line," ");
    		sscanf(pch,"(%f,%f)",&obst[count][0][0],&obst[count][0][1]);

    		int j=1;
    		for(int i=0; i<obstacle_size-1; i++){
    			pch = strtok(NULL, " ");
				  sscanf(pch,"(%f,%f)",&obst[count][j][0],&obst[count][j][1]);
     			j++;
  			}
  			obst[count][j][0] = obst[count][0][0];
    		obst[count][j][1] = obst[count][0][1];
  		}
		if(count>= num_obstacles && count<num_obstacles+num_regions){
  			const char* pch = strtok(line," ");
    		sscanf(pch,"(%f,%f)",&goal[count-num_obstacles][0][0],&goal[count-num_obstacles][0][1]);

    		int j=1;
    		for(int i=0; i<region_size-1; i++){
    			pch = strtok(NULL, " ");
				  sscanf(pch,"(%f,%f)",&goal[count-num_obstacles][j][0],&goal[count-num_obstacles][j][1]);
     			j++;
  			}
  			goal[count-num_obstacles][j][0] = goal[count-num_obstacles][0][0];
    		goal[count-num_obstacles][j][1] = goal[count-num_obstacles][0][1];
  		}
  		if(count==num_obstacles+num_regions){
  			const char* pch = strtok(line," ");
    		sscanf(pch,"(%f,%f)",&environ[0][0][0],&environ[0][0][1]);
    		for(int i=1; i<4; i++){
    			pch = strtok(NULL, " ");
				sscanf(pch,"(%f,%f)",&environ[0][i][0],&environ[0][i][1]);
  			}
  			environ[0][4][0] = environ[0][0][0];
    		environ[0][4][1] = environ[0][0][1];
  		}
  		if(count==num_obstacles+num_regions+1){
  			sscanf(line,"(%f,%f)",&loc[0],&loc[1]);
  		}

  		count++;
  	}
  	fclose(f);
}