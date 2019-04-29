# LTL-RRT* Implementation C
Savini Prematilleke

## 1. OBTAIN INPUTS 
<NBA text file>  
text file that contains the buchi automata generated from the input LTL formula<br/>
- use the LTL2BA software at http://www.lsv.fr/~gastin/ltl2ba/ <br/>  
- input an LTL formula where every atomic propoisition is labelled as follows:<br/>  
  for example with 2 robots and 3 goal regions, there are 6 atomic propositions <br/> 
  p1: robot1 in region1  <br/>
  p2: robot1 in region2   <br/>
  p3: robot1 in region3  <br/>
  p4: robot2 in region1  <br/>
  p5: robot2 in region2  <br/>
  p6: robot2 in region3  <br/>
- make sure the "use verbose mode" box is checked before clicking convert  <br/>
- copy and paste the "Buchi automaton after simplification" output into a text file <br/> 
- see "NBA.txt" which is a sample text file for the LTL formula:  <br/>
  "F(p4)&&GF(p3&&(Fp1))&&(!p1Up2)&&F(!p5)"  <br/>

<environment text file>   <br/>
text file that dictates the location and shape of obstacles, goal regions, and the bounds of the environment  <br/>
- each line must contain the vertices of one shape in counter-clockwise order, each vertex 
  must be of the form "(0.0,0.0)" and separated by a single space  <br/>
- the first lines contain the vertices of the obstacles, the next lines contains the 
  vertices of the goals, the second to last line contains the vertices of the environment 
  bounds (note: the environment must be a rectangle), and the last line contains the 
  initial position of the robots (note: all robots must have the same initial location).   <br/>
- see "environ.txt" which is a sample text file for a 1x1 environment, with 2 rectangular 
  obstacles and 6 triangular goal regions  <br/>

<# of robots>  
integer specifying the number of robots in the simulation  

<# of goal regions>  
integer specifying the number of goal regions in the configuration space  

<goal size>   
integer specifying the number of vertices in goal region polygon (note: all goal regions 
must have the same number of vertices)  

<# of obstacles>   
integer specifying the number of obstacles in the configuration space  

<obstacle size>  
integer specifying the number of vertices in obstacle polygon (note: all obstacles must 
have the same number of vertices)  

<# of prefix runs>   
integer specifying maximum number of iterations for prefix run  

<# of suffix runs>  
integer specifying maximum number of iterations for suffix run  

## 2. COMPILE
download repository  

compile program
~~~~
make
~~~~

## 3. EXECUTING 
run the program
~~~~
./main <NBA text file> <environment text file> <# of robots> <# of goal regions> <goal size> <# of obstacles> <obstacle size> <# of prefix runs> <# of suffix runs>
~~~~
to output all feasible and least-cost prefix and suffix paths, and corresponding run times

run the program
~~~~
python plotPath.py
~~~~
to visualize the least-cost prefix and suffix paths
- currently this program will only plot for the environment specified in "environ.txt"

## REFERENCES 
Based on the work of user XushengLuo
