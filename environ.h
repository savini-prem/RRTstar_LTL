#ifndef __ENVIRON_H__
#define __ENVIRON_H__

float*** initPolygon(int num_polygons, int num_vertices);
void fillPolygon(char* line, float** shapes);
void printPolygon(float*** shape, int num_polygons, int num_vertices);
void getEnvironment(float* loc, char* filename, float*** environ, float*** obst, int num_obstacles, int obstacle_size, float*** goal, int num_regions, int region_size);

#endif