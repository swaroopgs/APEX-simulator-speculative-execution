---------------------------------------------------------------------------------
APEX Pipeline Simulator
---------------------------------------------------------------------------------
An implementation of 7 Stage APEX out-of-order Pipeline with speculative execution

Author :
---------------------------------------------------------------------------------
Swaroop Gowdra Shanthakumar

Notes:
----------------------------------------------------------------------------------
1) This codes implements a simulator for the simulation of an out-of-order processor which supports speculative execution.
2) Supports speculative excecution, data forwarding, branching, functional units, memory instructions  and utilizes ROB, LSQ, Issue Queue to speed up execution.  

File-Info:
----------------------------------------------------------------------------------
1) Makefile 			- To make the compile and run process easier
2) file_parser.c 	- Contains Functions to parse input file. No need to change this file
3) cpu.c          - Contains Implementation of APEX cpu. You can edit as needed
4) cpu.h          - Contains various data structures declarations needed by 'cpu.c'. You can edit as needed\

How to compile and run
----------------------------------------------------------------------------------
1) go to terminal, cd into project directory and type 'make' to compile project
2) Run using ./apex_sim <input file name> \'93simulate\'94/\'93run\'94 <number_of_cycles>

