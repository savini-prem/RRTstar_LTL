#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include "ltl.h"
#include "rrtstar.h"

#define LINE_SIZE 100
#define TRUE 1
#define FALSE 0

BA_t * initBA(carray_t * s){
  BA_t *  B = malloc(sizeof(*B));
  B->trans = malloc(s->len * sizeof(*B->trans));
  int dim = s->len;
  for(int i = 0; i < dim; i++){
    B->trans[i] = malloc(s->len * sizeof(*B->trans));
  }
  for(int i = 0 ; i < dim ; i++){
    for(int j = 0 ; j < dim; j++){
      B->trans[i][j] = malloc(sizeof(*B->trans[i][j]));
			B->trans[i][j]->arr = NULL;
      B->trans[i][j]->len = 0;
    }
  }
  B->DIM = dim; 
  return B;
}

void printBA(BA_t * B){
  for(int i = 0; i < B->DIM; i++){
    for(int j = 0; j < B->DIM; j++){
      printf("START : %d END : %d : ", i,j);
      for(int k = 0; k < B->trans[i][j]->len; k++){
	      printf("%d ",B->trans[i][j]->arr[k]);
      }
      printf("\n");
    }
  }
}

void label(BA_t*  B,carray_t * s, carray_t * a, char * filename,int numAPs){
  FILE * f = fopen(filename, "r");
  char* line = (char*)malloc(LINE_SIZE*sizeof(char));
  char* tokena = (char*)malloc(LINE_SIZE*sizeof(char)); 
  char* tokenb = (char*)malloc(LINE_SIZE*sizeof(char));

  int start = -1;
  int end = 0;
  int* result;

  rewind(f);
  while(fgets(line, LINE_SIZE, f) != NULL){
    //printf("line: %s", line);

    if(strstr(line, "state")){
      start++;
    }
    else{
      //token = strsep(&line, "{} -> ");
      tokena = strsep(&line, "-"); 
      tokenb = strsep(&tokena, "{"); 
      
      // now find end index
      end = getStateIndex(s,line);

      iarray_t* APS = containsAPs(tokenb,numAPs);

      iarray_t* APSnot = containsAPsNOT(tokena,numAPs);

      if(strchr(tokenb,'p')==NULL && APSnot->len==0){ // case 1: must be 1
        result = (int*)malloc(a->len*sizeof(int));
        for(int i =0; i<a->len; i++){
          result[i] = i;
        }
        B->trans[start][end]->arr = result;
        B->trans[start][end]->len = a->len;
      }
      //else fillTrans(a,APS,APSnot,B->trans[start][end]); 
      else{
        int index = 0; 
        int* result = (int*)malloc(0*sizeof(int));

        for(int i=0; i<a->len; i++){
          int counter = 0;
          for(int j=0; j<APS->len; j++){
            if(a->arr[i][APS->arr[j]]=='1') counter++;
          }
          for(int j=0; j<APSnot->len; j++){
            if(a->arr[i][APSnot->arr[j]]=='0') counter++;
          }
          if(counter == APS->len + APSnot->len) {
            index++; 
            result = realloc(result, index*sizeof(int));
            result[index-1] = i;
          }
        }

        B->trans[start][end]->arr = result;
        B->trans[start][end]->len = index;
      }
    }
  }
}

// returns index of state array that matches token 
int getStateIndex(carray_t* s, char* token){
  for(int i=0; i<s->len; i++){
    if(strstr(token,s->arr[i])!=NULL) return i;
  }
  return -1;
}

// returns tarray of alphabet indices for alphabet entries that 
// contain the integer specifying the AP of interest
// also returns length of tarray 
void fillTrans(carray_t* a, iarray_t* APS, iarray_t* APSnot, iarray_t* trans){ //lmao most inefficient code ever fix this 
  int index = 0; 
  int* result = (int*)malloc(0*sizeof(int));

  for(int i=0; i<a->len; i++){
    int counter = 0;
    for(int j=0; j<APS->len; j++){
      if(a->arr[i][APS->arr[j]]=='1') {
        counter++;
      }
    }
    for(int j=0; j<APSnot->len; j++){
      if(a->arr[i][APSnot->arr[j]]=='0') {
        counter++;
      }
    }
    if(counter == APS->len + APSnot->len) {
      index++; 
      result = realloc(result, index*sizeof(int));
      result[index-1] = i;
    }
  }

  trans->arr = result; 
  trans->len = index; 
}

// returns array of APs contained in token
iarray_t* containsAPs(char* token, int numAPs){
  iarray_t* result = (iarray_t*)malloc(sizeof(iarray_t));
  int counter = 0; 
  for(int i=1; i<numAPs+1; i++){
    if(strchr(token,i+'0')){
      counter++;
    }
  }
  result->arr = (int*)malloc(counter*sizeof(int));
  counter = 0;
  for(int i=1; i<numAPs+1; i++){
    if(strchr(token,i+'0')){
      counter++;
      result->arr[counter-1] = i-1;
    }
  }
  result->len = counter;
  return result;
}

//round 2
iarray_t* containsAPsNOT(char* token, int numAPs){
  iarray_t* result = (iarray_t*)malloc(sizeof(iarray_t));
  int start,end;
  int count = 0; 
  for(int i=1; i<numAPs+1; i++){
    if(strchr(token,i+'0')){
      count++; 
      if(count == 1) start = i; 
      if(count == 2) end = i;  
    }
  }
  if(count == 0) result->len = 0; 
  if(count == 1) {
    result->arr = (int*)malloc((numAPs-start)*sizeof(int));
    result->len = numAPs-start; 
    for(int i=0; i<numAPs-start; i++) result->arr[i] = i+start; 
  }
  if(count == 2) {
    result->arr = (int*)malloc((numAPs-(end-start+1))*sizeof(int));
    result->len = numAPs-(end-start+1); 
    int counter = 0;
    int index = 0;
    while(index<numAPs-(end-start+1)){
      counter++; 
      if(counter<start || counter>end) {
        result->arr[index] = counter-1; 
        index++; 
      }
    }
  }
  return result;
}

// given location of robots, returns index that indicates which APs are true
// currently this code works assuming there is only one robot l m a o (otherwise #APs!=#goals)
int whichAPIsTrue(carray_t* a, int numAPs, vertex_t* new_steer, float*** goal, int num_goals, int goal_size){
  char word[numAPs];
  for(int k=0; k<new_steer->pos->len; k++){
    for(int j=0; j<num_goals; j++){
      if(!collisionFree(new_steer->pos->coord[k], goal[j], goal_size)){
        strcpy(&word[k*num_goals+j],"1");
      }
      else strcpy(&word[k*num_goals+j],"0");
    }
  }
  //printf("word: %s\n", word);
  for(int i=0; i<a->len; i++){
    if(strcmp(word,a->arr[i])==0) return i;
  }

  return -1; // should never happen
}

// given current state and array of which APS are true
// returns array of next possible states
iarray_t* nextBState(int current_state, int alpha_index, BA_t* B){
  iarray_t* result = newIArray(); 
  for(int i=0; i<B->DIM; i++){
    if(arrayContains(alpha_index,B->trans[current_state][i]) && !arrayContains(i,result)){
      //printf("index: %d, ", i);
      addToIArray(result,i);
      //printf("%d\n",result->arr[result->len-1]); 
    }
  }
  return result;
}

int isConnectionValid(int near_arri_bstate, iarray_t* new_next_states){
  for(int j=0; j<new_next_states->len; j++){
    if(new_next_states->arr[j]==near_arri_bstate) return 1; 
  }
  return 0; 
}

iarray_t* getSuffixGoals(BA_t* B, int goal){
  iarray_t* result = newIArray();
  for(int i=0; i<B->DIM; i++){
    if(B->trans[i][goal]->len != 0) addToIArray(result, i);
  }
  return result;
}

// returns TRUE if array contains a particular value 
int arrayContains(int alpha_index, iarray_t* trans){ // CHANGE TO ARRAY 
  for(int i=0; i<trans->len; i++){
    if(trans->arr[i] == alpha_index) return TRUE;
  }
  return FALSE;
}

iarray_t* newIArray(void){
  iarray_t* result = (iarray_t*)malloc(sizeof(iarray_t));
    result->arr = NULL;
    result->len = 0; 
    return result;
}

void addToIArray(iarray_t* result, int element){
  result->arr = (int*)realloc(result->arr,(result->len+1)*sizeof(int));
  result->arr[result->len] = element;
  result->len++;
}

// old states: 
iarray_t* findAcceptStates(carray_t* s){
  iarray_t* result = newIArray();
  for(int i=0; i<s->len; i++){
    if(strstr(s->arr[i], "accept")){
      addToIArray(result,i);
    }
  }
  return result;
}

carray_t * createState(void){
  carray_t * s = malloc(sizeof(*s));
  s->len = 0;
  s->arr = NULL;
  return s;
}

void addState(carray_t * s, char * line){
  char * temp = strdup(line);
  if(strstr(line, "state")){
    s->len++;
    s->arr = realloc(s->arr,s->len * sizeof(*(s->arr)));
    strtok(temp, " ");
    s->arr[s->len - 1] = strdup(strtok(NULL,"\n")); 
  }
  free(temp);
}

void printStates(carray_t * s){
  printf("STATES:\n");
  for(int i = 0; i < s->len; i++){
    printf("%s\n", s->arr[i]);
  }
}

void freeStates(carray_t * s){
  for(int i = 0; i < s->len; i++){
    free(s->arr[i]);
  }
  free(s->arr);
  free(s);
}

carray_t * readStates(char * filename){
  FILE * f = fopen(filename, "r");
  char line[LINE_SIZE];
  if(f == NULL){
    fprintf(stderr, "failure to open");
    exit(EXIT_FAILURE);
  }
  carray_t * s = createState(); 
  while(fgets(line, LINE_SIZE, f) != NULL){
    addState(s,line);
  }
  if(fclose(f) != 0){
    fprintf(stderr, "failure to close");
    exit(EXIT_FAILURE);
  }
  return s;
}

// old alpha: 
char* dec2bin(int x, int numAPs){
  char* bin = (char*)malloc(numAPs+1*sizeof(char));

  for(int i=0; i<numAPs; i++){
    bin[numAPs-1-i] = (x % 2) + '0';
    x /= 2;
  }
  bin[numAPs] = '\0';
  return bin;
}

carray_t* createAlpha(int numAPs, int num_robs, int num_regions){
  carray_t* a = malloc(sizeof(*a));
  a->len = 0; 
  a->arr = malloc(pow(2, numAPs) * sizeof(*(a->arr))); 
   
  for(int i = 0; i < pow(2, numAPs); i++){
    char* word = dec2bin(i,numAPs);

    int count[num_robs]; 
    for(int j=0; j<num_robs; j++) count[j] = 0; 
    
    for(int k=0; k<num_robs; k++){
      for(int j=0; j<num_regions; j++){
        if(word[k*num_regions+j]-'1'==0) count[k]++; 
      }
    }

    int flag = 0; 
    for(int j=0; j<num_robs; j++) if(count[j]>=2) flag = 1; 

    if(flag==0){
      a->arr[a->len] = word;
      a->len++;
    }
  }
  return a;
}

void printAlpha(carray_t * alpha){
  for(int i = 0; i < alpha->len; i++){
    printf("%s\n", alpha->arr[i]);
  }
}