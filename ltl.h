#ifndef __LABEL_H__
#define __LABEL_H__

#include "rrtstar.h"

typedef struct _iarray_t{
  int* arr;
  int len;
} iarray_t;

typedef struct _carray_t{
	int len;
	char** arr; 
} carray_t;

typedef struct _BA_t{
  int DIM;
  iarray_t*** trans;
} BA_t;

BA_t * initBA(carray_t * s);
void printBA(BA_t * B);

iarray_t* newIArray(void);
void addToIArray(iarray_t* result, int element);


void label(BA_t * B,carray_t *s, carray_t *a, char * filename, int aps);

int getStateIndex(carray_t * s, char* token);
void getAlphabetIndex(carray_t* a, iarray_t* APS, iarray_t* trans);

iarray_t* containsAPs(char* token, int numAPs);

int whichAPIsTrue(carray_t* a, int numAPs, float loc[], float goal[][5][2], int num_goals, int goal_size);

iarray_t* nextBState(int current_state, int alpha_index, BA_t* B);
int isConnectionValid(int near_arri_bstate, iarray_t* new_next_states);
iarray_t* getSuffixGoals(BA_t* B, int goal);

int arrayContains(int alpha_index, iarray_t* trans); // CHANGE TO ARRAY 


// old states:
iarray_t* findAcceptStates(carray_t* s);
void addState(carray_t* s, char* lines);
void printStates(carray_t* s);
void freeStates(carray_t* s);
carray_t* readStates(char* filename);

// old alpha: 
char* dec2bin(int x, int len);
carray_t* createAlpha(int len);
void printAlpha(carray_t* alpha);

#endif
