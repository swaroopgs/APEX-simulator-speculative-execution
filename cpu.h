#ifndef _APEX_CPU_H_
#define _APEX_CPU_H_
/**
 *  cpu.h
 *  Contains various CPU and Pipeline Data structures
 *
 *  Author :
 *  Gaurav Kothari (gkothar1@binghamton.edu)
 *  State University of New York, Binghamton
 */
enum
{
	F,
	DRF,
	IQ,
	INT1,
	INT2,
	WLSQ,
	MUL1,
	MUL2,
	MUL3,
	BRANCH,
	MEM,
	NUM_STAGES
};

enum FU
{
	INT,
	MUL,
	BN_Z
};

/* Format of an APEX instruction  */
typedef struct APEX_Instruction
{
	char opcode[128];	// Operation Code
	int rd;		    // Destination Register Address
	int rs1;		    // Source-1 Register Address
	int rs2;			//// Source-2 Register Address
	int rs3;		    // Source-3 Register Address
	int imm;		    // Literal Value
	int stage_finished;
} APEX_Instruction;

/* Format of an ROB ENTRY  */
typedef struct ROB_ENTRY
{
	int pc_value; //Address of the instruction
	int arch_register; //where to store the pc_value
	int result; //result
	int exception_codes;
	int result_valid;
	char instruction_type[128];
	int phys_register;
} ROB_ENTRY;

typedef struct BIS_ENTRY
{
	int pc_value;
	int rob_index;
	int checkpoint_entry;
} BIS_ENTRY;

typedef struct BTB_ENTRY
{
	int branch_ins_pc_value;//key
	int target_pc_value;//address to jump to
	int history_bit;//for prediction
} BTB_ENTRY;

typedef struct LSQ_ENTRY
{
	int value;
	int src1_tag;
	int src1_valid;
	int load_dest_reg;
	int calculated_mem_address;
	int address_valid;
	int ins_type;
	int rob_index;//rob_index
	int cycle_counter;
	int pc;
	int bis_index;
} LSQ_ENTRY;

typedef struct IQ_ENTRY
{
	// Has an LSQ index in case of LOAD/STORE instructions
	// Has a BIS index to most recent Branch instruction
	int lsq_index;//or a lsq_index, which ever is easier to implement
	int bis_index;//index of the most recent branch instruction, so as to flush all instructions in all stages that are processed after a mispredicted branch
	int rob_index;
	int des_physical_reg;
	int src2_value;
	int src2_tag;
	int src2_ready;
	int src1_value;
	int src1_tag;
	int src1_ready;
	int literal;
	int fu_type_needed;
	char opcode[128];
	int stage_finished;
	int pc_value;
} IQ_ENTRY;

/* Model of CPU stage latch */
typedef struct CPU_Stage
{
	int pc;		    // Program Counter
	char opcode[128];	// Operation Code
	int rs1;		    // Source-1 Register Address
	int rs2;		// Source-2 Register Address
	int rs3;		//Source-3 Register Address
	int rd;		    // Destination Register Address
	int imm;		    // Literal Value
	int rs1_value;	// Source-1 Register Value
	int rs2_value;	// Source-2 Register Value
	int rs3_value; //Source-3 Register Value
	int buffer;		// Latch to hold some value
	int mem_address;	// Computed Memory Address
	int busy;		    // Flag to indicate, stage is performing some action
	int stalled;		// Flag to indicate, stage is stalled
	int is_empty;
	IQ_ENTRY iq_entry;
	
} CPU_Stage;

/* Model of APEX CPU */
typedef struct APEX_CPU
{
	/* Clock cycles elasped */
	int clock;

	/* Current program counter */
	int pc;

	/* Integer register file */
	int regs[16];
	int regs_valid[16];

	/*Physical Register file with its AR values and list to indicate if PR is free or not*/
	int phys_regs[24];
	int phys_regs_valid[24];
	// Index represents the PR and values 1- represents free,0 - represents occupied.
	int free_PR_list[24];
	int free_PR_list_checkpoint_1[24];
	int free_PR_list_checkpoint_2[24];
	//This is to hold flag condition flag for PR if any. 

	int flag_condition[24];
	//Number of consumers for every physical register.
	int consumers[24];

	//Rename table to contain info with Index represents the AR and values represents the Physical Register.
	int rename_table[16];

	//NOTE: check if this is correct??
	//Two extra array for branching purposes(checkpoint predict) still not clear why , need to check!
	int checkpoint_rename_table_1[16];
	int checkpoint_rename_table_2[16];

	//NOTE: Check if this is correct??
	/* Array of 9 CPU_stage */
	CPU_Stage stage[9];

	/* Code Memory where instructions are stored */
	APEX_Instruction* code_memory;
	int code_memory_size;

	/* Data Memory */
	int data_memory[4000];

	/* Some stats */
	int ins_completed;
	IQ_ENTRY IQ[8];
	ROB_ENTRY ROB[12];
	LSQ_ENTRY LSQ[6];
	BTB_ENTRY BTB[8];
	BIS_ENTRY BIS[2];

	int rob_head;
	int rob_tail;

	int lsq_head;
	int lsq_tail;

	int bis_head;
	int bis_tail;

	int btb_tail;
	int checkpoint1_free;
	int checkpoint2_free;
	
	int iq_free[8];
	int latest_arithmetic_inst_phys_reg;
	int execution_started;
	int lsq_current_size;
	int rob_current_size;
	int bis_current_size;
} APEX_CPU;

//physical registers, free list of physical registers, architectural registers, rename table, 2 checkpoint rename table.
//head, tail for LSQ to implment a queue in an array.
//Array for IQ free or implement a 

APEX_Instruction*
create_code_memory(const char* filename, int* size);

APEX_CPU*
APEX_cpu_init(const char* filename);

int
APEX_cpu_run(APEX_CPU *cpu, int no_of_cycles, int flag);

void
APEX_cpu_stop(APEX_CPU* cpu);

int
fetch(APEX_CPU* cpu);

int
decode(APEX_CPU* cpu);

int
execute(APEX_CPU* cpu);

int
memory(APEX_CPU* cpu);

int
writeback(APEX_CPU* cpu);

#endif
