## RRTstar-LTL in C 
Implementation of a sampling-based LTL motion planning algorithm that incrementally builds trees that explore the continuous product state-space. Both probabilistically complete and asymptotically optimal. 

## Instructions: 
1. Clone this repository and copy all code
2. Using http://www.lsv.fr/~gastin/ltl2ba/index.php in verbose mode, obtain a text file of the NBA produced by the LTL formula of choice
3. Compile using gcc compiler
4. Run main.c (inputs: NBA text file, # of APs, # of prefix runs, # of suffix runs; output: coordinates of all feasible prefix paths including least-cost path, coordinates of all feasible suffix paths for every prefix path including least-cost path)
5. Run plot.py (output: plot of obstacles, goal region, and all paths)

## References: 
Based on the work of user XushengLuo
