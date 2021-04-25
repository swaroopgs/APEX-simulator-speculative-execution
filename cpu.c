/*
 *  cpu.c
 *  Contains APEX cpu pipeline implementation
 *
 *  Author :
 *  Gaurav Kothari (gkothar1@binghamton.edu)
 *  State University of New York, Binghamton
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cpu.h"

/* Set this flag to 1 to enable debug messages */
int ENABLE_DEBUG_MESSAGES = 1;

BTB_ENTRY* mispredicted_branch_btb_entry;
IQ_ENTRY* mispredicted_branch_iq_entry;
int mispredicted_branch_target_address_new;
int flush_and_reload = 0;
int stop_fetch_decode = 0;

const int LSQ_SIZE = 6;
const int BTB_SIZE = 8;
const int BIS_SIZE = 2;
const int ROB_SIZE = 12;
const int IQ_SIZE = 8;
const int PRF_SIZE = 24;
const int ARF_SIZE = 16;

/*
 * This function creates and initializes APEX cpu.
 *
 * Note : You are free to edit this function according to your
 * 				implementation
 */
APEX_CPU *
APEX_cpu_init(const char *filename)
{
	if (!filename) {
		return NULL;
	}

	APEX_CPU *cpu = malloc(sizeof(*cpu));
	if (!cpu) {
		return NULL;
	}

	/* Initialize PC, Registers and all pipeline stages */
	cpu->pc = 4000;
	memset(cpu->regs, -1, sizeof(int) * 16);
	memset(cpu->regs_valid, 1, sizeof(int) * 16);
	memset(cpu->phys_regs, 0, sizeof(int) * 24);
	memset(cpu->phys_regs_valid, 1, sizeof(int) * 24);
	memset(cpu->free_PR_list, 1, sizeof(int) * 24);
	memset(cpu->free_PR_list_checkpoint_1, 1, sizeof(int) * 24);
	memset(cpu->free_PR_list_checkpoint_2, 1, sizeof(int) * 24);
	memset(cpu->flag_condition, -1, sizeof(int) * 24);
	memset(cpu->rename_table, -1, sizeof(int) * 16);
	memset(cpu->checkpoint_rename_table_1, -1, sizeof(int) * 16);
	memset(cpu->checkpoint_rename_table_1, -1, sizeof(int) * 16);
	memset(cpu->stage, 0, sizeof(CPU_Stage) * NUM_STAGES);
	memset(cpu->data_memory, 0, sizeof(int) * 4000);
	memset(cpu->consumers, 0, sizeof(int) * 24);
	//memset(cpu->iq_free, 1, sizeof(int) * IQ_SIZE);
	memset(cpu->ROB, 0, sizeof(ROB_ENTRY) * ROB_SIZE);
	memset(cpu->IQ, 0, sizeof(IQ_ENTRY) * IQ_SIZE);
	memset(cpu->LSQ, 0, sizeof(LSQ_ENTRY) * LSQ_SIZE);
	memset(cpu->BTB, 0, sizeof(BTB_ENTRY) * BTB_SIZE);
	memset(cpu->BIS, 0, sizeof(BIS_ENTRY) * BIS_SIZE);
	cpu->rob_tail = cpu->lsq_tail = cpu->bis_tail = cpu->rob_head = cpu->lsq_head = cpu->bis_head = -1;
	cpu->rob_current_size = cpu->lsq_current_size = cpu->bis_current_size = cpu->btb_tail = 0;
	cpu->checkpoint1_free = cpu->checkpoint2_free = 1;
	memset(cpu->iq_free, 1, sizeof(int) * IQ_SIZE);

	/* Parse input file and create code memory */
	cpu->code_memory = create_code_memory(filename, &cpu->code_memory_size);

	if (!cpu->code_memory) {
		free(cpu);
		return NULL;
	}

	if (ENABLE_DEBUG_MESSAGES) {
		fprintf(stderr,
				"APEX_CPU : Initialized APEX CPU, loaded %d instructions\n",
				cpu->code_memory_size);
		fprintf(stderr, "APEX_CPU : Printing Code Memory\n");
		printf("%-9s %-9s %-9s %-9s %-9s\n", "opcode", "rd", "rs1", "rs2", "imm");

		for (int i = 0; i < cpu->code_memory_size; ++i) {
			printf("%-9s %-9d %-9d %-9d %-9d\n",
				   cpu->code_memory[i].opcode,
				   cpu->code_memory[i].rd,
				   cpu->code_memory[i].rs1,
				   cpu->code_memory[i].rs2,
				   cpu->code_memory[i].imm);
		}
	}

	/* Make all stages busy except Fetch stage, initally to start the pipeline*/
	for (int i = 1; i < NUM_STAGES; ++i) {
		cpu->stage[i].busy = 1;
	}

	return cpu;
}

/*
 * This function de-allocates APEX cpu.
 *
 * Note : You are free to edit this function according to your
 * 				implementation
 */
void APEX_cpu_stop(APEX_CPU *cpu)
{
	free(cpu->code_memory);
	free(cpu);
}

/* Converts the PC(4000 series) into
 * array index for code memory
 *
 * Note : You are not supposed to edit this function
 *
 */
int get_code_index(int pc)
{
	return (pc - 4000) / 4;
}

static void
print_instruction(CPU_Stage *stage, APEX_CPU* cpu, IQ_ENTRY* iq_entry, int from_stage)
{
	char opcode[128];
	strcpy(opcode, (from_stage > DRF ? iq_entry->opcode : stage->opcode));
	if (strcmp(opcode, "STORE") == 0) {
		if(from_stage == F) {
			printf("%s,R%d,R%d,#%d ", opcode, stage->rs1, stage->rs2, stage->imm);
		} else {
			APEX_Instruction *current_ins = &cpu->code_memory[get_code_index(iq_entry->pc_value)];
			printf("%s,R%d,R%d,#%d ", opcode, current_ins->rs1, current_ins->rs2, current_ins->imm);
		}
		if(iq_entry) {
			LSQ_ENTRY* lsq_entry = &cpu->LSQ[iq_entry->lsq_index];
			printf("[%s,P%d,P%d,#%d]", opcode, lsq_entry->src1_tag, iq_entry->src1_tag, iq_entry->literal);
		}	
	} else if (strcmp(opcode, "MOVC") == 0) {
		if(from_stage == F) {
			printf("%s,R%d,#%d ", opcode, stage->rd, stage->imm);
		} else {
			APEX_Instruction *current_ins = &cpu->code_memory[get_code_index(iq_entry->pc_value)];
			printf("%s,R%d,#%d ", opcode, current_ins->rd, current_ins->imm);
		}
		if(iq_entry) {
			printf("[%s,P%d,#%d]", opcode, iq_entry->des_physical_reg, iq_entry->literal);
		}
	} else if (strcmp(opcode, "ADD") == 0 || strcmp(opcode, "SUB") == 0 || strcmp(opcode, "MUL") == 0 || strcmp(opcode, "LDR") == 0 || strcmp(opcode, "AND") == 0 || strcmp(opcode, "OR") == 0 || strcmp(opcode, "EX-OR") == 0) {
		if(from_stage == F) {
			printf("%s,R%d,R%d,R%d ", opcode, stage->rd, stage->rs1, stage->rs2);
		} else {
			APEX_Instruction *current_ins = &cpu->code_memory[get_code_index(iq_entry->pc_value)];
			printf("%s,R%d,R%d,R%d ", opcode, current_ins->rd, current_ins->rs1, current_ins->rs2);
		}
		if(iq_entry) {
			printf("[%s,P%d,P%d,P%d]", opcode, iq_entry->des_physical_reg, iq_entry->src1_tag, iq_entry->src2_tag);
		}	
	} else if (strcmp(opcode, "LOAD") == 0 || strcmp(opcode, "ADDL") == 0 || strcmp(opcode, "SUBL") == 0) {
		if(from_stage == F) {
			printf("%s,R%d,R%d,#%d ", opcode, stage->rd, stage->rs1, stage->imm);
		} else {
			APEX_Instruction *current_ins = &cpu->code_memory[get_code_index(iq_entry->pc_value)];
			printf("%s,R%d,R%d,#%d ", opcode, current_ins->rd, current_ins->rs1, current_ins->imm);
		}
		if(iq_entry) {
			printf("[%s,P%d,P%d,#%d]", opcode, iq_entry->des_physical_reg, iq_entry->src1_tag, iq_entry->literal);
		}	
	} else if (strcmp(opcode, "STR") == 0) {	
		if(from_stage == F) {
			printf("%s,R%d,R%d,R%d ", opcode, stage->rs1, stage->rs2, stage->rs3);
		} else {
			APEX_Instruction *current_ins = &cpu->code_memory[get_code_index(iq_entry->pc_value)];
			printf("%s,R%d,R%d,R%d ", opcode, current_ins->rs1, current_ins->rs2, current_ins->rs3);
		}
		if(iq_entry) {
			LSQ_ENTRY* lsq_entry = &cpu->LSQ[iq_entry->lsq_index];
			printf("[%s,P%d,P%d,P%d]", opcode, lsq_entry->src1_tag, iq_entry->src1_tag, iq_entry->src2_tag);
		}	
	} else if (strcmp(opcode, "BZ") == 0 || strcmp(opcode, "BNZ") == 0) {
		if(from_stage == F) {
			printf("%s,#%d", opcode, stage->imm);
		} else {
		}
		if(iq_entry) {
			printf("[%s,#%d]", opcode, iq_entry->literal);
		}
	} else if (strcmp(opcode, "JUMP") == 0) {
		if(from_stage == F) {
			printf("%s,R%d,#%d ", opcode, stage->rs1, stage->imm);
		} else {
			APEX_Instruction *current_ins = &cpu->code_memory[get_code_index(iq_entry->pc_value)];
			printf("%s,R%d,#%d ", opcode, current_ins->rs1, iq_entry->literal);
		}
		if(iq_entry) {
			printf("[%s,P%d,#%d]", opcode, iq_entry->src1_tag, iq_entry->literal);
		}
	} else if (strcmp(opcode, "HALT") == 0 || strcmp(opcode, "HALT\n") == 0) {
		printf("%s", opcode);
	} else if (strcmp(opcode, "NOP") == 0) {
		printf(" NOP");
	} else if (stage->pc == 0 && ((strcmp(opcode, "") == 0) || (strcmp(opcode, "EMPTY") == 0))) {
		strcpy(stage->opcode, "EMPTY");
		printf(" EMPTY");
	}
}

/* Debug function which dumps the cpu stage
 * content
 *
 * Note : You are not supposed to edit this function
 *
 */
static void
print_stage_content(char *name, CPU_Stage *stage, int is_active, APEX_CPU* cpu, IQ_ENTRY* iq_entry, int from_stage)
{
	printf("%-15s ", name);
	if (is_active)
	{
		//fprintf(stderr, "Test print stage 1 pc value-> %d", stage);
		if(stage) {
			printf("(I%d)", get_code_index(from_stage > DRF ? iq_entry->pc_value : stage->pc));
		}
		print_instruction(stage, cpu, iq_entry, from_stage);
		//fprintf(stderr, "Test print stage 2");
	}
	else
	{
		printf("EMPTY");
	}
	printf("\n");
}

static void 
print_lsq(APEX_CPU* cpu, LSQ_ENTRY* lsq_entry) {
	APEX_Instruction *current_ins = &cpu->code_memory[get_code_index(lsq_entry->pc)];
	if (strcmp(current_ins->opcode, "STORE") == 0) {
		printf("%s,R%d,R%d,#%d ", current_ins->opcode, current_ins->rs1, current_ins->rs2, current_ins->imm);
		//printf("%s,P%d,P%d,#%d ", current_ins->opcode, lsq_entry->src1_tag, cpu->rename_table[current_ins->rs2], current_ins->imm);
	} else if(strcmp(current_ins->opcode, "STR") == 0) {
		printf("%s,R%d,R%d,R%d ", current_ins->opcode, current_ins->rs1, current_ins->rs2, current_ins->rs3);
		//printf("%s,P%d,P%d,P%d ", current_ins->opcode, lsq_entry->src1_tag, cpu->rename_table[current_ins->rs2], cpu->rename_table[current_ins->rs3]);
	} else if(strcmp(current_ins->opcode, "LOAD") == 0) {
		printf("%s,R%d,R%d,R%d ", current_ins->opcode, current_ins->rd, current_ins->rs1, current_ins->imm);
		//printf("%s,R%d,R%d,#%d ", current_ins->opcode, lsq_entry->load_dest_reg, cpu->rename_table[current_ins->rs2], iq_entry->literal);
	} else if(strcmp(current_ins->opcode, "LDR") == 0) {
		printf("%s,R%d,R%d,R%d ", current_ins->opcode, current_ins->rd, current_ins->rs1, current_ins->rs2);
		//printf("%s,R%d,R%d,#%d ", iq_entry->opcode, iq_entry->des_physical_reg, iq_entry->src1_tag, iq_entry->src2_tag);
	}
}

static void
print_rob(APEX_CPU* cpu, int pc_value) {
	APEX_Instruction *current_ins = &cpu->code_memory[get_code_index(pc_value)];
	if (strcmp(current_ins->opcode, "STORE") == 0) {
    	printf("%s,R%d,R%d,#%d ", current_ins->opcode, current_ins->rs1, current_ins->rs2, current_ins->imm);
		//printf("[%s,P%d,P%d,#%d]", iq_entry->opcode, lsq_entry->src1_tag, iq_entry->src1_tag, iq_entry->literal);
  	} else if (strcmp(current_ins->opcode, "STR") == 0) {
    	printf("%s,R%d,R%d,R%d ", current_ins->opcode, current_ins->rs1, current_ins->rs2, current_ins->rs3);
		//printf("[%s,P%d,P%d,P%d]", iq_entry->opcode, lsq_entry->src1_tag, iq_entry->src1_tag, iq_entry->src2_tag);
  	} else if (strcmp(current_ins->opcode, "ADD") == 0 || strcmp(current_ins->opcode, "SUB") == 0 || strcmp(current_ins->opcode, "MUL") == 0 || strcmp(current_ins->opcode, "AND") == 0 ||
      strcmp(current_ins->opcode, "OR") == 0 || strcmp(current_ins->opcode, "EX-OR") == 0) {
    	printf("%s,R%d,R%d,R%d ", current_ins->opcode, current_ins->rd, current_ins->rs1, current_ins->rs2);
		//printf("[%s,P%d,P%d,P%d]", iq_entry->opcode, iq_entry->des_physical_reg, iq_entry->src1_tag, iq_entry->src2_tag);
   	} else if (strcmp(current_ins->opcode, "ADDL") == 0 || strcmp(current_ins->opcode, "SUBL") == 0 || strcmp(current_ins->opcode, "LOAD") == 0) {
     	printf("%s,R%d,R%d,#%d ", current_ins->opcode, current_ins->rd, current_ins->rs1, current_ins->imm);
		//printf("[%s,P%d,P%d,#%d]", current_ins->opcode, iq_entry->des_physical_reg, iq_entry->src1_tag, iq_entry->literal);
  	} else if (strcmp(current_ins->opcode, "MOVC") == 0) {
    	printf("%s,R%d,#%d ", current_ins->opcode, current_ins->rd, current_ins->imm);
		//printf("[%s,P%d,#%d]", current_ins->opcode, iq_entry->des_physical_reg, iq_entry->literal);
  	} else if (strcmp(current_ins->opcode, "BZ") == 0 || strcmp(current_ins->opcode, "BNZ") == 0) {
    	printf("%s,#%d", current_ins->opcode, current_ins->imm);
  	} else if (strcmp(current_ins->opcode, "JUMP") == 0) {
    	printf("%s,R%d,#%d", current_ins->opcode, current_ins->rs1, current_ins->imm);
		//printf("[%s,P%d,#%d]", iq_entry->opcode, iq_entry->src1_tag, iq_entry->literal);
  	} else if (strcmp(current_ins->opcode, "HALT") == 0) {
    	printf("%s", current_ins->opcode);
  	}
}

int free_physical_registers(APEX_CPU* cpu, int rs1, int rs2, int rs3) {
	int i,j;
	for(i = 0; i < PRF_SIZE; i++) {
		if(i == rs1 || i == rs2 || i == rs3) {
			continue;
		}
		for(j=0; j < ARF_SIZE; j++) {
			if(cpu->rename_table[j] == i) {
				break;
			}
		}
		if(j == ARF_SIZE && cpu->consumers[i] == 0) {
			cpu->free_PR_list[i] = 1;
		}
	}
	return 0;
}
/*
 *  Fetch Stage of APEX Pipeline
 *
 *  Note : You are free to edit this function according to your
 * 				 implementation
 */
int fetch(APEX_CPU *cpu)
{
	CPU_Stage *stage = &cpu->stage[F];
	stage->is_empty = 0;
	stage->stalled = 0;
	fprintf(stderr, "Not yet segmentation fault -1\n");
	if (!stop_fetch_decode && !stage->busy && !stage->stalled)
	{
		/* Store current PC in fetch latch */
		stage->pc = cpu->pc;
		fprintf(stderr, "Not yet segmentation fault %d\n", stage->pc);
		/* Index into code memory using this pc and copy all instruction fields into
		 * fetch latch
		 */
		APEX_Instruction *current_ins = &cpu->code_memory[get_code_index(cpu->pc)];
		strcpy(stage->opcode, current_ins->opcode);
		stage->rd = current_ins->rd;
		stage->rs1 = current_ins->rs1;
		stage->rs2 = current_ins->rs2;
		stage->rs3 = current_ins->rs3;
		stage->imm = current_ins->imm;

		/* Copy data from fetch latch to decode latch*/
		if (!(&cpu->stage[DRF])->stalled) {
			cpu->stage[DRF] = cpu->stage[F];
			current_ins->stage_finished = F;
			int i;
			for (i = 0; i < BTB_SIZE; i++) {
				if (cpu->BTB[i].branch_ins_pc_value == stage->pc && cpu->BTB[i].history_bit == 1) {
					cpu->pc = cpu->BTB[i].target_pc_value; // Take the branch
					break;
				}
			}
			if(i == BTB_SIZE) {
				cpu->pc += 4;
			}
		} else {
			fprintf(stderr, "Not yet segmentation fault -2\n");
			stage->stalled = 1;
		}
	}
	stage->is_empty = 1;
	fprintf(stderr, "Not yet segmentation fault\n");
	if (ENABLE_DEBUG_MESSAGES){
		print_stage_content("Instruction at FETCH_____STAGE--->\t", stage, !stage->stalled && get_code_index(stage->pc) < cpu->code_memory_size, cpu, NULL, F);
	}
			fprintf(stderr, "Not yet segmentation fault 2\n");
	return 0;
}

/*
 *  Decode Stage of APEX Pipeline
 *
 *  Note : You are free to edit this function according to your
 * 				 implementation
 */
int decode(APEX_CPU *cpu)
{
	CPU_Stage *stage = &cpu->stage[DRF];
	stage->stalled = 0;
	stage->is_empty = 0;
	APEX_Instruction *current_ins = &cpu->code_memory[get_code_index(stage->pc)];
	if (!stop_fetch_decode && cpu->clock > 0 && !stage->busy && !stage->stalled && strcmp(stage->opcode, "NOP") != 0 && current_ins->stage_finished < DRF)
	{
		/* Read data from register file for store */
		int is_stage_stalled = 0;
		if (cpu->rob_current_size == ROB_SIZE) {
			fprintf(stderr, "Stage stalled at decode rob\n");
			is_stage_stalled = 1;
		}
		if (!is_stage_stalled && (strcmp(stage->opcode, "BZ") == 0 || strcmp(stage->opcode, "BNZ") == 0)) {
			if (cpu->bis_current_size == BIS_SIZE) {
				fprintf(stderr, "Stage stalled at decode bis\n");
				is_stage_stalled = 1;
			}
		}
		if (!is_stage_stalled && (strcmp(stage->opcode, "STORE") == 0 || strcmp(stage->opcode, "STR") == 0 || strcmp(stage->opcode, "LOAD") == 0 || strcmp(stage->opcode, "LDR") == 0)) {
			if (cpu->lsq_current_size == LSQ_SIZE) {
				fprintf(stderr, "Stage stalled at decode lsq\n");
				is_stage_stalled = 1;
			}
		}
		IQ_ENTRY *iq_entry = NULL;
		int i;
		for (i = 0; i < IQ_SIZE; i++) {
			if (cpu->iq_free[i] >= 1) {
				//iq_entry = &cpu->IQ[i];
				//cpu->iq_free[i] = 0;
				break;
			}
		}
		if (i == IQ_SIZE) {
			is_stage_stalled = 1;
		}
		int first_free_phy_reg = -1;
		int previous_phy_reg = -1;
		int rs1_physical = stage->rs1 > -1 ? cpu->rename_table[stage->rs1] : -1;
		int rs2_physical = stage->rs2 > -1 ? cpu->rename_table[stage->rs2] : -1;
		int rs3_physical = stage->rs3 > -1 ? cpu->rename_table[stage->rs3] : -1;
		if(!is_stage_stalled) {
			free_physical_registers(cpu, rs1_physical, rs2_physical, rs3_physical);
		}
		if (!(strcmp(stage->opcode, "STORE") == 0 || strcmp(stage->opcode, "STR") == 0 || strcmp(stage->opcode, "HALT") == 0 || strcmp(stage->opcode, "BZ") == 0 || strcmp(stage->opcode, "BNZ") == 0 || strcmp(stage->opcode, "JUMP") == 0)) {
			for (i = 0; i < PRF_SIZE; i++) {
				if (cpu->free_PR_list[i]) {
					first_free_phy_reg = i;
					//cpu->free_PR_list[i] = 0;
					//cpu->phys_regs_valid[i] = 0;
					break;
				}
			}
			if(!(first_free_phy_reg > -1)) {
				is_stage_stalled = 1;
			}
		}
		//fprintf(stderr, "Test Decode 1\n");
		if (is_stage_stalled) {
			stage->stalled = 1;
		} else {
			if (!(strcmp(stage->opcode, "STORE") == 0 || strcmp(stage->opcode, "STR") == 0 || strcmp(stage->opcode, "HALT") == 0 || strcmp(stage->opcode, "BZ") == 0 || strcmp(stage->opcode, "BNZ") == 0 || strcmp(stage->opcode, "JUMP") == 0)) {
				for (i = 0; i < PRF_SIZE; i++) {
					if (cpu->free_PR_list[i]) {
						first_free_phy_reg = i;
						cpu->free_PR_list[i] = 0;
						cpu->phys_regs_valid[i] = 0;
						break;
					}
				}
				if(first_free_phy_reg > -1) {
					//fprintf(stderr, "first  free  physical register %d %s\n", first_free_phy_reg, stage->opcode);
					previous_phy_reg = cpu->rename_table[stage->rd];
					cpu->rename_table[stage->rd] = first_free_phy_reg;
				}
			}
			//fprintf(stderr, "Test Decode 2\n");
			cpu->rob_tail = (cpu->rob_tail + 1) % ROB_SIZE;
			cpu->rob_current_size += 1;
			ROB_ENTRY *rob_entry = &cpu->ROB[cpu->rob_tail];
			if(cpu->rob_head == -1) {
				cpu->rob_head = cpu->rob_tail;
			}
			LSQ_ENTRY *lsq_entry;
			BIS_ENTRY *bis_entry;

			rob_entry->arch_register = stage->rd;
			rob_entry->exception_codes = 0;
			rob_entry->result_valid = 0;
			rob_entry->result = 0;
			rob_entry->pc_value = stage->pc;
			strcpy(rob_entry->instruction_type, stage->opcode);
			rob_entry->phys_register = first_free_phy_reg;
			cpu->execution_started = 1;
			//fprintf(stderr, "Test Decode 3\n");
			if(strcmp(stage->opcode, "HALT") == 0) {
				stage->stalled = 1;
				(&cpu->stage[F])->stalled = 1;
				stop_fetch_decode = 1;
				rob_entry->result_valid = 1;
			}else {
				//fprintf(stderr, "Test Decode 4\n");
				if (strcmp(stage->opcode, "STORE") == 0 || strcmp(stage->opcode, "STR") == 0) {
					cpu->lsq_tail = (cpu->lsq_tail + 1) % LSQ_SIZE;
					cpu->lsq_current_size += 1;
					if(cpu->lsq_head == -1) {
						cpu->lsq_head = cpu->lsq_tail;
					}
					lsq_entry = &cpu->LSQ[cpu->lsq_tail];
					lsq_entry->address_valid = 0;
					lsq_entry->calculated_mem_address = 0;
					lsq_entry->ins_type = 1;
					lsq_entry->rob_index = cpu->rob_tail;
					lsq_entry->src1_tag = rs1_physical;
					lsq_entry->value = cpu->phys_regs[rs1_physical];
					lsq_entry->src1_valid = cpu->phys_regs_valid[rs1_physical];
					lsq_entry->bis_index = cpu->bis_tail;
					lsq_entry->pc = stage->pc;
				} else if (strcmp(stage->opcode, "LOAD") == 0 || strcmp(stage->opcode, "LDR") == 0) {
					cpu->lsq_tail = (cpu->lsq_tail + 1) % LSQ_SIZE;
					lsq_entry = &cpu->LSQ[cpu->lsq_tail];
					lsq_entry->address_valid = 0;
					lsq_entry->calculated_mem_address = 0;
					lsq_entry->ins_type = 0;
					lsq_entry->rob_index = cpu->rob_tail;
					lsq_entry->load_dest_reg = first_free_phy_reg;
					lsq_entry->bis_index = cpu->bis_tail;
					lsq_entry->pc = stage->pc;
				}
				//fprintf(stderr, "Test Decode 5\n");
				if (strcmp(stage->opcode, "BZ") == 0 || strcmp(stage->opcode, "BNZ") == 0) {
					cpu->bis_tail = (cpu->bis_tail + 1) % BIS_SIZE;
					if(cpu->bis_head == -1) {
						cpu->bis_head = cpu->bis_tail;
					}
					bis_entry = &cpu->BIS[cpu->bis_tail];
					//fprintf(stderr, "Test Decode 51\n");
					bis_entry->pc_value = stage->pc;
					//fprintf(stderr, "Test Decode 5111\n");
					bis_entry->rob_index = cpu->rob_tail;
					
					int is_present_in_btb = 0;
					for (int i = 0; i < BTB_SIZE; i++) {
						if ((&cpu->BTB[i])->branch_ins_pc_value == stage->pc) {
							is_present_in_btb = 1;
							break;
						}
					}
					//fprintf(stderr, "Test Decode 52\n");
					if (!is_present_in_btb) {
						BTB_ENTRY* btb_entry = &cpu->BTB[cpu->btb_tail];
						btb_entry->branch_ins_pc_value = stage->pc;
						btb_entry->history_bit = 0;
						cpu->btb_tail += 1;
					}
				}
				//fprintf(stderr, "Test decode 6\n");
				for (i = 0; i < IQ_SIZE; i++) {
					if (cpu->iq_free[i] >= 1) {
						iq_entry = &cpu->IQ[i];
						cpu->iq_free[i] = 0;
						break;
					}
				}
				strcpy(iq_entry->opcode, stage->opcode);
				if(strcmp(stage->opcode, "STR") == 0) {
					cpu->consumers[rs2_physical] += 1;
					cpu->consumers[rs3_physical] += 1;
				} else if(strcmp(stage->opcode, "BZ") == 0 || strcmp(stage->opcode, "BNZ") == 0) {
					cpu->consumers[cpu->latest_arithmetic_inst_phys_reg] += 1;
				} else if(strcmp(stage->opcode, "ADD") == 0 || strcmp(stage->opcode, "ADDL") == 0 || strcmp(stage->opcode, "SUB") == 0 || strcmp(stage->opcode, "AND") == 0 || strcmp(stage->opcode, "OR") == 0 || strcmp(stage->opcode, "EX-OR") == 0 || strcmp(stage->opcode, "SUBL") == 0 || strcmp(stage->opcode, "MUL") == 0 || strcmp(stage->opcode, "LDR") == 0) {
					cpu->consumers[rs1_physical] += 1;
					cpu->consumers[rs2_physical] += 1;
				} else if(strcmp("STORE", stage->opcode) == 0) {
					cpu->consumers[rs2_physical] += 1;
				} else if(strcmp("JUMP", stage->opcode) == 0 || strcmp("LOAD", stage->opcode) == 0) {
					cpu->consumers[rs1_physical] += 1;
				} 
				iq_entry->src1_value = (strcmp(stage->opcode, "STR") == 0 || strcmp(stage->opcode, "STORE") == 0) ? cpu->phys_regs[rs2_physical] : ((strcmp(stage->opcode, "BZ") == 0 || strcmp(stage->opcode, "BNZ") == 0) ? cpu->flag_condition[cpu->latest_arithmetic_inst_phys_reg] : cpu->phys_regs[rs1_physical]);
				iq_entry->src2_value = (strcmp(stage->opcode, "STR") == 0) ? cpu->phys_regs[rs3_physical] : cpu->phys_regs[rs2_physical];
				iq_entry->src2_tag = (strcmp(stage->opcode, "STR") == 0) ? rs3_physical : rs2_physical;
				iq_entry->src1_tag = (strcmp(stage->opcode, "STR") == 0 || strcmp(stage->opcode, "STORE") == 0) ? rs2_physical : ((strcmp(stage->opcode, "BZ") == 0 || strcmp(stage->opcode, "BNZ") == 0) ? cpu->latest_arithmetic_inst_phys_reg : rs1_physical);
				iq_entry->src2_ready = (strcmp(stage->opcode, "STR") == 0) ? cpu->phys_regs_valid[rs3_physical] : cpu->phys_regs_valid[rs2_physical];
				iq_entry->src1_ready = (strcmp(stage->opcode, "STR") == 0 || strcmp(stage->opcode, "STORE") == 0) ? cpu->phys_regs_valid[rs2_physical] : ((strcmp(stage->opcode, "BZ") == 0 || strcmp(stage->opcode, "BNZ") == 0) ? cpu->phys_regs_valid[cpu->latest_arithmetic_inst_phys_reg] : cpu->phys_regs_valid[rs1_physical]);
				iq_entry->lsq_index = cpu->lsq_tail;
				iq_entry->literal = stage->imm;
				iq_entry->bis_index = cpu->bis_tail;
				iq_entry->des_physical_reg = first_free_phy_reg;
				iq_entry->rob_index = cpu->rob_tail;
				iq_entry->stage_finished = DRF;
				iq_entry->pc_value = stage->pc;
				if (strcmp(stage->opcode, "MUL") == 0) {
					iq_entry->fu_type_needed = MUL;
				} else if (strcmp(stage->opcode, "BZ") == 0 || strcmp(stage->opcode, "BNZ") == 0) {
					iq_entry->fu_type_needed = BRANCH;
					// Checkpointing here
					//1. Check which checkpoint rename table is free
					//2. Copy contents of the rename table to the checkpoint rename table
					//3. Copy contents of free list to the checkppoint free list table
					int i;
					if(cpu->checkpoint1_free == 1) {
						bis_entry->checkpoint_entry = 1;
						for(i = 0; i < ARF_SIZE; i++) {
							cpu->checkpoint_rename_table_1[i] = cpu->rename_table[i];
						}
						for(i = 0; i < PRF_SIZE; i++) {
							cpu->free_PR_list_checkpoint_1[i] = cpu->free_PR_list[i];
						}
					} else {
						bis_entry->checkpoint_entry = 2;
						for(i = 0; i < ARF_SIZE; i++) {
							cpu->checkpoint_rename_table_2[i] = cpu->rename_table[i];
						}
						for(i = 0; i < PRF_SIZE; i++) {
							cpu->free_PR_list_checkpoint_2[i] = cpu->free_PR_list[i];
						}
					}
				} else if(strcmp(stage->opcode, "JUMP") == 0) {
					iq_entry->fu_type_needed = BRANCH;
					stage->stalled = 1;
					(&cpu->stage[F])->stalled = 1;

				} else {
					iq_entry->fu_type_needed = INT;
				}
				if(strcmp("ADD", stage->opcode) == 0 || strcmp("SUB", stage->opcode) == 0 || strcmp("ADDL", stage->opcode) == 0 || strcmp("SUBL", stage->opcode) == 0 || strcmp("MUL", stage->opcode) == 0) {
					cpu->latest_arithmetic_inst_phys_reg = first_free_phy_reg;
				}
				if(previous_phy_reg > -1 && cpu->consumers[previous_phy_reg] <= 0) {
					cpu->free_PR_list[previous_phy_reg] = 1;
				}
			}
		}
		//fprintf(stderr, "Test decode 7\n");
		current_ins->stage_finished = DRF;
		if (ENABLE_DEBUG_MESSAGES) {
			print_stage_content("Instruction at DECODE_RF_STAGE--->\t", stage, (!stage->stalled && current_ins->stage_finished == DRF && (get_code_index(stage->pc) < cpu->code_memory_size)), cpu, iq_entry, DRF);
		}
		//TODO: Handle tracking of the latest arithmetic instruction for branch instructions.
		//TODO: Handle flushing and rollback, forwarding, instruction commitment and freeing physical registers
	}
	else if (ENABLE_DEBUG_MESSAGES) {
		print_stage_content("Instruction at DECODE_RF_STAGE--->\t", stage, 0, cpu, NULL, DRF);
	}
	printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
	printf("Details of RENAME TABLE State --\n");
	for (int i = 0; i < 16; i++) {
		//To display content of rename table only when PR is assigned to AR.
		if (cpu->rename_table[i] != -1) {
			printf("R[%d] -> P[%d]\n", i, cpu->rename_table[i]);
		}
	}
	printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
	//cpu->stage[IQ] = cpu->stage[DRF];

	printf("Details of ARF State –\n");
	for(int i = 0; i< 16; i++) {
		if(cpu->regs[i] != -1) {
			printf("R%d --> %d\n", i, cpu->regs[i]);
		}
	}
	printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
	stage->is_empty = 1;
	return 0;
}

/*
 *  Memory Stage of APEX Pipeline
 *
 *  Note : You are free to edit this function according to your
 * 				 implementation
 */
int issue_queue(APEX_CPU *cpu)
{
	// CPU_Stage *stage = &cpu->stage[IQ];
	int i;
	int selected_inst_pc_value_int = 2000000;
	int selected_inst_pc_value_mul = 2000000;
	int selected_inst_pc_value_branch = 2000000;

	int int_fu_issued = -1;
	int mul_fu_issued = -1;
	int branch_fu_issued = -1;

	IQ_ENTRY selected_int_inst;
	IQ_ENTRY selected_mul_inst;
	IQ_ENTRY selected_branch_inst;
	for (i = 0; i < IQ_SIZE; i++) {
		if(cpu->iq_free[i] != 0) {
			continue;
		}
		fprintf(stderr, "cpu free %d %d\n", i, cpu->iq_free[i]);
		IQ_ENTRY iq_entry = cpu->IQ[i];
		if(strcmp("ADD", iq_entry.opcode) == 0 || strcmp("SUB", iq_entry.opcode) == 0 || strcmp("MUL", iq_entry.opcode) == 0 || strcmp("AND", iq_entry.opcode) == 0 || strcmp("OR", iq_entry.opcode) == 0 || strcmp("EX-OR", iq_entry.opcode) == 0 || strcmp("LDR", iq_entry.opcode) == 0 || strcmp("STR", iq_entry.opcode) == 0) {
			if (iq_entry.src1_ready == 1 && iq_entry.src2_ready == 1) {
				if (iq_entry.fu_type_needed == INT && iq_entry.pc_value < selected_inst_pc_value_int) {
					selected_int_inst = iq_entry;
					selected_inst_pc_value_int = iq_entry.pc_value;
					int_fu_issued = i;
				} else if (iq_entry.fu_type_needed == MUL && iq_entry.pc_value < selected_inst_pc_value_mul) {
					mul_fu_issued = i;
					selected_mul_inst = iq_entry;
					selected_inst_pc_value_mul = iq_entry.pc_value;
				} else if (iq_entry.fu_type_needed == BRANCH && iq_entry.pc_value < selected_inst_pc_value_branch) {
					branch_fu_issued = i;
					selected_branch_inst = iq_entry;
					selected_inst_pc_value_branch = iq_entry.pc_value;
				}
			}
		} else if(strcmp("ADDL", iq_entry.opcode) == 0 || strcmp("SUBL", iq_entry.opcode) == 0 || strcmp("JUMP", iq_entry.opcode) == 0 || strcmp("LOAD", iq_entry.opcode) == 0 || strcmp("STORE", iq_entry.opcode) == 0 || strcmp("BZ", iq_entry.opcode) == 0 || strcmp("BNZ", iq_entry.opcode) == 0) {
			if (iq_entry.src1_ready == 1) {
				if (iq_entry.fu_type_needed == INT && iq_entry.pc_value < selected_inst_pc_value_int) {
					selected_int_inst = iq_entry;
					selected_inst_pc_value_int = iq_entry.pc_value;
					int_fu_issued = i;
				} else if (iq_entry.fu_type_needed == MUL && iq_entry.pc_value < selected_inst_pc_value_mul) {
					mul_fu_issued = i;
					selected_mul_inst = iq_entry;
					selected_inst_pc_value_mul = iq_entry.pc_value;
				} else if (iq_entry.fu_type_needed == BRANCH && iq_entry.pc_value < selected_inst_pc_value_branch) {
					branch_fu_issued = i;
					selected_branch_inst = iq_entry;
					selected_inst_pc_value_branch = iq_entry.pc_value;
				}
			}
		} else {
			if (iq_entry.fu_type_needed == INT && iq_entry.pc_value < selected_inst_pc_value_int) {
				selected_int_inst = iq_entry;
				selected_inst_pc_value_int = iq_entry.pc_value;
				int_fu_issued = i;
			} else if (iq_entry.fu_type_needed == MUL && iq_entry.pc_value < selected_inst_pc_value_mul) {
				mul_fu_issued = i;
				selected_mul_inst = iq_entry;
				selected_inst_pc_value_mul = iq_entry.pc_value;
			} else if (iq_entry.fu_type_needed == BRANCH && iq_entry.pc_value < selected_inst_pc_value_branch) {
				branch_fu_issued = i;
				selected_branch_inst = iq_entry;
				selected_inst_pc_value_branch = iq_entry.pc_value;
			}
		}
	}
	if(ENABLE_DEBUG_MESSAGES) {
		printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
		printf("Details of IQ (Issue Queue) State –\n");
		IQ_ENTRY* iq_entry_1;
		for(i = 0; i < IQ_SIZE; i++) {
			if(cpu->iq_free[i] == 0) {
				fprintf(stderr, "IQ[0%d] --> ", i);
				iq_entry_1 = &cpu->IQ[i];
				printf("IQ[0%d] --> ",i);
				print_stage_content("", NULL, 1, cpu, iq_entry_1, IQ);
			}
		}
		printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");

		printf("Details of LSQ (Load-Store Queue) State –\n");
		LSQ_ENTRY* lsq_entry;
		int j = 0;
		if(cpu->lsq_current_size > 0) {
			int counter = cpu->lsq_current_size;
			for(j = cpu->lsq_head; counter > 0 ; j = (j+1)%LSQ_SIZE) {
				lsq_entry = &cpu->LSQ[j];
				printf("LSQ[0%d] --> ",j);
				print_lsq(cpu, lsq_entry);
				printf("\n");
				counter -= 1;
			}
		}
		printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");

		printf("Details of ROB (Reorder Buffer) State –\n");
		ROB_ENTRY* rob_entry;
		if(cpu->rob_current_size > 0) {
			int counter = cpu->rob_current_size;
			for(j = cpu->rob_head; counter > 0 ; j = (j+1)%ROB_SIZE) {
				rob_entry = &cpu->ROB[j];
				printf("ROB[0%d] --> ",j);
				print_rob(cpu, rob_entry->pc_value);
				printf("\n");
				counter -= 1;
			}
		}
		printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
	}
	if(int_fu_issued > -1) {
		IQ_ENTRY entry = selected_int_inst;
		(&cpu->stage[INT1])->iq_entry = entry;
		(cpu->stage[INT1]).iq_entry.stage_finished = IQ;
		(cpu->stage[INT1]).busy = 0;
		(cpu->stage[INT1]).stalled = 0;
		cpu->iq_free[int_fu_issued] = 1;
	}
	if(mul_fu_issued > -1) {
		(&cpu->stage[MUL1])->iq_entry = (selected_mul_inst);
		(&cpu->stage[MUL1])->iq_entry.stage_finished = IQ;
		(&cpu->stage[MUL1])->busy = 0;
		(&cpu->stage[MUL1])->stalled = 0;
		cpu->iq_free[mul_fu_issued] = 1;
	}
	if(branch_fu_issued > -1) {
		(&cpu->stage[BRANCH])->iq_entry = (selected_branch_inst);
		(&cpu->stage[BRANCH])->iq_entry.stage_finished = IQ;
		(&cpu->stage[BRANCH])->busy = 0;
		(&cpu->stage[BRANCH])->stalled = 0;
		cpu->iq_free[branch_fu_issued] = 1;
	}

	return 0;
}

int int_fu_1(APEX_CPU *cpu)
{
	CPU_Stage *stage = &cpu->stage[INT1];
	IQ_ENTRY* iq_entry = &stage->iq_entry;
	if (iq_entry && !stage->busy && !stage->stalled && iq_entry->stage_finished < INT1)
	{
		if(strcmp(iq_entry->opcode,"MOVC") == 0) {
			stage->buffer = iq_entry->literal;
		} else if (strcmp(iq_entry->opcode, "ADD") == 0) {
			stage->buffer = iq_entry->src1_value + iq_entry->src2_value;
		} else if (strcmp(iq_entry->opcode, "ADDL") == 0) {
			stage->buffer = iq_entry->src1_value + iq_entry->literal;
		} else if (strcmp(iq_entry->opcode, "SUB") == 0) {
			stage->buffer = iq_entry->src1_value - iq_entry->src2_value;
		} else if (strcmp(iq_entry->opcode, "SUBL") == 0) {
			stage->buffer = iq_entry->src1_value - iq_entry->literal;
		} else if (strcmp(iq_entry->opcode, "AND") == 0) {
			stage->buffer = iq_entry->src1_value & iq_entry->src2_value;
		} else if (strcmp(iq_entry->opcode, "OR") == 0) {
			stage->buffer = iq_entry->src1_value | iq_entry->src2_value;
		} else if (strcmp(iq_entry->opcode, "EX-OR") == 0) {
			stage->buffer = iq_entry->src1_value ^ iq_entry->src2_value;
		} else if (strcmp(iq_entry->opcode, "LOAD") == 0) {
			stage->mem_address = iq_entry->src1_value + iq_entry->literal;
		} else if (strcmp(iq_entry->opcode, "LDR") == 0) {
			stage->mem_address = iq_entry->src1_value + iq_entry->src2_value;;
		} else if (strcmp(iq_entry->opcode, "STORE") == 0) {
			stage->mem_address = iq_entry->src1_value + iq_entry->literal;
		} else if (strcmp(iq_entry->opcode, "STR") == 0) {
			stage->mem_address = iq_entry->src1_value + iq_entry->src2_value;;
		}
		iq_entry->stage_finished = INT1;
		cpu->stage[INT2] = cpu->stage[INT1];
		if (ENABLE_DEBUG_MESSAGES) {
			print_stage_content("Instruction at INT1_FU_STAGE--->", stage, iq_entry->stage_finished == INT1, cpu, iq_entry, INT1);
		}
	}
	else if (ENABLE_DEBUG_MESSAGES) {
		print_stage_content("Instruction at INT1_FU_STAGE--->", stage, 0, cpu, iq_entry, INT1);
	}
	return 0;
}

int int_fu_2(APEX_CPU *cpu)
{
	CPU_Stage *stage = &cpu->stage[INT2];
	IQ_ENTRY* iq_entry = &stage->iq_entry;
	if (iq_entry && !stage->busy && !stage->stalled && iq_entry->stage_finished < INT2)
	{
		if(strcmp(iq_entry->opcode, "STR") == 0 || strcmp(iq_entry->opcode, "STORE") == 0 || strcmp(iq_entry->opcode, "LOAD") == 0 || strcmp(iq_entry->opcode, "LDR") == 0) {
			cpu->stage[WLSQ] = cpu->stage[INT2];
		} else if (strcmp(iq_entry->opcode, "ADD") == 0 || strcmp(iq_entry->opcode, "ADDL") == 0 || strcmp(iq_entry->opcode, "SUB") == 0 || strcmp(iq_entry->opcode, "SUBL") == 0 || strcmp(iq_entry->opcode, "MOVC") == 0 || strcmp(iq_entry->opcode, "AND") == 0 || strcmp(iq_entry->opcode, "OR") == 0 || strcmp(iq_entry->opcode, "EX-OR") == 0){
			ROB_ENTRY *rob_entry = &cpu->ROB[iq_entry->rob_index];	
			rob_entry->exception_codes = 0;
			rob_entry->result_valid = 1;
			rob_entry->result = stage->buffer;
			cpu->phys_regs[iq_entry->des_physical_reg] = stage->buffer;
			cpu->phys_regs_valid[iq_entry->des_physical_reg] = 1;
			if (strcmp(stage->opcode, "ADD") == 0 || strcmp(stage->opcode, "ADDL") == 0 || strcmp(stage->opcode, "SUB") == 0 || strcmp(stage->opcode, "SUBL") == 0) {
				cpu->flag_condition[iq_entry->des_physical_reg] = (stage->buffer == 0);
			}
			int i;
			for (i = 0; i < IQ_SIZE; i++) {
				IQ_ENTRY *test_iq_entry = &cpu->IQ[i];
				if (cpu->iq_free[i] == 0) {
					if (iq_entry->des_physical_reg == test_iq_entry->src1_tag) {
						if(strcmp(test_iq_entry->opcode, "BZ") == 0 || strcmp(test_iq_entry->opcode, "BNZ") == 0) {
							test_iq_entry->src1_value = (stage->buffer == 0);
						} else {
							test_iq_entry->src1_value = stage->buffer;
						}
						test_iq_entry->src1_ready = 1;
					}
					if (iq_entry->des_physical_reg == test_iq_entry->src2_tag) {
						test_iq_entry->src2_value = stage->buffer;
						test_iq_entry->src2_ready = 1;
					}
				}
			}
			//Forward to LSQ entries
			if(cpu->lsq_current_size > 0) {
				int counter = cpu->lsq_current_size;
				for(i = cpu->lsq_head; counter > 0; i = (i+1)%LSQ_SIZE) {
					LSQ_ENTRY* lsq_entry = &(cpu->LSQ[i]);
					if(lsq_entry->src1_tag == iq_entry->des_physical_reg) {
						lsq_entry->src1_valid = 1;
						lsq_entry->value = stage->buffer;
					}
					counter -= 1;
				}
			}
		}
		if (strcmp(stage->opcode, "ADD") == 0 || strcmp(stage->opcode, "SUB") == 0 || strcmp(stage->opcode, "AND") == 0 || strcmp(stage->opcode, "OR") == 0 || strcmp(stage->opcode, "EX-OR") == 0 || strcmp(stage->opcode, "LDR") == 0 || strcmp(stage->opcode, "STR") == 0) {
			cpu->consumers[iq_entry->src1_tag] -= 1;
			cpu->consumers[iq_entry->src2_tag] -= 1;
		} else if (strcmp(stage->opcode, "ADDL") == 0 || strcmp(stage->opcode, "SUBL") == 0 || strcmp(stage->opcode, "LOAD") == 0 || strcmp(stage->opcode, "STORE") == 0) {
			cpu->consumers[iq_entry->src1_tag] -= 1;
		}
		iq_entry->stage_finished = INT2;
		if (ENABLE_DEBUG_MESSAGES) {
			print_stage_content("Instruction at INT2_FU_STAGE--->", stage, iq_entry->stage_finished == INT2, cpu, iq_entry, INT2);
		}
	}
	else if (ENABLE_DEBUG_MESSAGES) {
		print_stage_content("Instruction at INT2_FU_STAGE--->", stage, 0, cpu, iq_entry, INT2);
	}
	return 0;
}

int writeToLSQ(APEX_CPU *cpu) {
	CPU_Stage *stage = &cpu->stage[WLSQ];
	IQ_ENTRY* iq_entry = &stage->iq_entry;
	if(iq_entry && !stage->busy && !stage->stalled && iq_entry->stage_finished < WLSQ) {
		(&cpu->LSQ[iq_entry->lsq_index])->calculated_mem_address = stage->mem_address;
		(&cpu->LSQ[iq_entry->lsq_index])->address_valid = 1;
		iq_entry->stage_finished = WLSQ;
	}
	return 0;
}

int mul_fu_1(APEX_CPU *cpu)
{
	CPU_Stage *stage = &cpu->stage[MUL1];
	IQ_ENTRY* iq_entry = &stage->iq_entry;
	if (iq_entry && !stage->busy && !stage->stalled && iq_entry->stage_finished < MUL1) {
		if (strcmp(iq_entry->opcode, "MUL") == 0) {
				stage->buffer = iq_entry->src1_value * iq_entry->src2_value;
		}
		iq_entry->stage_finished = MUL1;
		cpu->stage[MUL2] = cpu->stage[MUL1];
		if (ENABLE_DEBUG_MESSAGES) {
			print_stage_content("Instruction at MUL1_FU_STAGE--->", stage, iq_entry->stage_finished == MUL1, cpu, iq_entry, MUL1);
		}
	}
	else if (ENABLE_DEBUG_MESSAGES) {
		print_stage_content("Instruction at MUL1_FU_STAGE--->", stage, 0, cpu, iq_entry, MUL1);
	}
	return 0;
}

int mul_fu_2(APEX_CPU *cpu)
{
	CPU_Stage *stage = &cpu->stage[MUL2];
	IQ_ENTRY* iq_entry = &stage->iq_entry;
	if (iq_entry && !stage->busy && !stage->stalled && iq_entry->stage_finished < MUL2) {
		iq_entry->stage_finished = MUL2;
		cpu->stage[MUL3] = cpu->stage[MUL2];
		if (ENABLE_DEBUG_MESSAGES) {
			print_stage_content("Instruction at MUL2_FU_STAGE--->", stage, iq_entry->stage_finished == MUL2, cpu, iq_entry, MUL2);
		}
	}
	else if (ENABLE_DEBUG_MESSAGES) {
		print_stage_content("Instruction at MUL2_FU_STAGE--->", stage, 0, cpu, iq_entry, MUL2);
	}
	return 0;
}

int mul_fu_3(APEX_CPU *cpu)
{
	CPU_Stage *stage = &cpu->stage[MUL3];
	IQ_ENTRY* iq_entry = &stage->iq_entry;
	if (iq_entry && !stage->busy && !stage->stalled && iq_entry->stage_finished < MUL3)
	{
		IQ_ENTRY* iq_entry1 = &stage->iq_entry;
		ROB_ENTRY *rob_entry = &cpu->ROB[iq_entry1->rob_index];	
		rob_entry->exception_codes = 0;
		rob_entry->result_valid = 1;
		rob_entry->result = stage->buffer;
		cpu->phys_regs[iq_entry->des_physical_reg] = stage->buffer;
		cpu->phys_regs_valid[iq_entry->des_physical_reg] = 1;
		int i;
		for (i = 0; i < IQ_SIZE; i++) {
			IQ_ENTRY *test_iq_entry = &cpu->IQ[i];
			if (cpu->iq_free[i] == 0) {
				if (iq_entry->des_physical_reg == test_iq_entry->src1_tag) {
					test_iq_entry->src1_value = stage->buffer;
					test_iq_entry->src1_ready = 1;
				}
				if (iq_entry->des_physical_reg == test_iq_entry->src2_tag) {
					test_iq_entry->src2_value = stage->buffer;
					test_iq_entry->src2_ready = 1;
				}
			}
		}
		//Forward to LSQ entries
		if(cpu->lsq_current_size > 0) {
			int counter = cpu->lsq_current_size;
			for(i = cpu->lsq_head; counter > 0; i = (i+1)%LSQ_SIZE) {
				LSQ_ENTRY* lsq_entry = &(cpu->LSQ[i]);
				if(lsq_entry->src1_tag == iq_entry->des_physical_reg) {
					lsq_entry->src1_valid = 1;
					lsq_entry->value = stage->buffer;
				}
				counter -= 1;
			}
		}
		cpu->consumers[iq_entry1->src1_tag] -= 1;
		cpu->consumers[iq_entry1->src2_tag] -= 1;
		iq_entry->stage_finished = MUL3;
		if (ENABLE_DEBUG_MESSAGES) {
			print_stage_content("Instruction at MUL3_FU_STAGE--->", stage, iq_entry->stage_finished == MUL3, cpu, iq_entry, MUL3);
		}
	}
	else if (ENABLE_DEBUG_MESSAGES) {
		print_stage_content("Instruction at MUL3_FU_STAGE--->", stage, 0, cpu, iq_entry, MUL3);
	}
	return 0;
	//TODO: Decrement the consumer count of the source physical registers
}

int branch_fu(APEX_CPU *cpu)
{
	CPU_Stage *stage = &cpu->stage[BRANCH];
	IQ_ENTRY* iq_entry = &stage->iq_entry;
	if(iq_entry && !stage->busy && !stage->stalled && iq_entry->stage_finished < BRANCH) {
		ROB_ENTRY *rob_entry = &cpu->ROB[iq_entry->rob_index];
		rob_entry->exception_codes = 0;
		rob_entry->result_valid = 1;
		stage->buffer = iq_entry->pc_value + iq_entry->literal;
		BTB_ENTRY* btb_entry;
		int i;
		if(strcmp(iq_entry->opcode, "BZ") == 0 || strcmp(iq_entry->opcode, "BNZ") == 0) {
			for(i = 0; i < BTB_SIZE; i++) {
				btb_entry = (&cpu->BTB[i]);
				if(btb_entry->branch_ins_pc_value == iq_entry->pc_value) {
					btb_entry->target_pc_value = stage->buffer;
					break;
				}
			}
			if(strcmp(iq_entry->opcode, "BZ") == 0 && ((iq_entry->src1_value == 1 && btb_entry->history_bit == 0) || (iq_entry->src1_value == 0 && btb_entry->history_bit == 1))) {
				//flush and go to target address
				int target_pc_value;
				if(btb_entry->history_bit == 0) {
					target_pc_value = btb_entry->target_pc_value;
				} else {
					target_pc_value = btb_entry->branch_ins_pc_value  + 4;
				}
				mispredicted_branch_btb_entry = btb_entry;
				mispredicted_branch_iq_entry = iq_entry;
				mispredicted_branch_target_address_new = target_pc_value;
				flush_and_reload = 1;
				stop_fetch_decode = 0;
			} else if(strcmp(iq_entry->opcode, "BNZ") == 0 && ((iq_entry->src1_value == 0 && btb_entry->history_bit == 0) || (iq_entry->src1_value == 1 && btb_entry->history_bit == 1))) {
				//flush and go to target address
				int target_pc_value;
				if(btb_entry->history_bit == 0) {
					target_pc_value = btb_entry->target_pc_value;
				} else {
					target_pc_value = btb_entry->branch_ins_pc_value  + 4;
				}
				mispredicted_branch_btb_entry = btb_entry;
				mispredicted_branch_iq_entry = iq_entry;
				mispredicted_branch_target_address_new = target_pc_value;
				flush_and_reload = 1;
				stop_fetch_decode = 0;
			}
			if(strcmp(iq_entry->opcode, "BZ") == 0) {
				if(iq_entry->src1_value == 1) {
					btb_entry->history_bit = 1;
				} else {
					btb_entry->history_bit = 0;
				}
			} else {
				if(strcmp(iq_entry->opcode, "BNZ") == 0) {
				if(iq_entry->src1_value == 0) {
					btb_entry->history_bit = 1;
				} else {
					btb_entry->history_bit = 0;
				}
			}
			}
		} else {
			//Inst is JUMP
			//flush and go to target address
			mispredicted_branch_btb_entry = NULL;
			mispredicted_branch_iq_entry = iq_entry;
			mispredicted_branch_target_address_new = iq_entry->src1_value + iq_entry->literal;
			flush_and_reload = 1;
			stop_fetch_decode = 0;
		}
		(&cpu->stage[F])->stalled = (&cpu->stage[DRF])->stalled = 0;
		iq_entry->stage_finished = BRANCH;
		if (ENABLE_DEBUG_MESSAGES) {
			print_stage_content("Instruction at BRANCH_FU_STAGE--->", stage, iq_entry->stage_finished == BRANCH, cpu, iq_entry, BRANCH);
		}
	}
	else if (ENABLE_DEBUG_MESSAGES) {
		print_stage_content("Instruction at BRANCH_FU_STAGE--->", stage, 0, cpu, iq_entry, BRANCH);
	}
	return 0;
}

int memory_issue(APEX_CPU *cpu) {
	if(cpu->lsq_head == -1) {
		//Nothing to do here
		return 0;
	}
	LSQ_ENTRY *lsq_entry = &cpu->LSQ[cpu->lsq_head];
	if(lsq_entry->rob_index == cpu->rob_head && lsq_entry->address_valid == 1 && ((lsq_entry->ins_type == 1 && lsq_entry->src1_valid == 1) || lsq_entry->ins_type == 0)) {
		if(lsq_entry->cycle_counter < 3) {
			lsq_entry->cycle_counter++;
		}
		if(lsq_entry->cycle_counter == 3) {
			ROB_ENTRY* rob_entry = &cpu->ROB[lsq_entry->rob_index];
			rob_entry->exception_codes = 0;
			rob_entry->result_valid = 1;
			if (lsq_entry->ins_type == 1) {
				cpu->data_memory[lsq_entry->calculated_mem_address] = lsq_entry->value;
			} else {
				rob_entry->result = cpu->data_memory[lsq_entry->calculated_mem_address];
				cpu->phys_regs[rob_entry->phys_register] = rob_entry->result;
				cpu->phys_regs_valid[rob_entry->phys_register] = 1;
				int i;
				for (i = 0; i < IQ_SIZE; i++) {
					IQ_ENTRY *iq_entry = &cpu->IQ[i];
					if (cpu->iq_free[i] == 0) {
						if (rob_entry->phys_register == iq_entry->src1_tag) {
							iq_entry->src1_value = rob_entry->result;
							iq_entry->src1_ready = 1;
						}
						if (rob_entry->phys_register == iq_entry->src2_tag) {
							iq_entry->src2_value = rob_entry->result;
							iq_entry->src2_ready = 1;
						}
					}
				}
				if((cpu->lsq_current_size - 1) > 0) {
					int counter = cpu->lsq_current_size - 1;
					for(i = cpu->lsq_head + 1; counter > 0; i = (i+1)%LSQ_SIZE) {
						LSQ_ENTRY* lsq_entry = &(cpu->LSQ[i]);
						if(lsq_entry->src1_tag == rob_entry->phys_register) {
							lsq_entry->src1_valid = 1;
							lsq_entry->value = rob_entry->result;
						}
						counter -= 1;
					}
				}
			}
			int next_head = cpu->lsq_head + 1;
			cpu->lsq_current_size -= 1;
			cpu->lsq_head = next_head % LSQ_SIZE;
		}
		if (ENABLE_DEBUG_MESSAGES) {
			printf("Instruction at MEM_FU_STAGE--->");
			print_lsq(cpu, lsq_entry);
			printf(" (Cycle %d)\n", lsq_entry->cycle_counter);
		}
	} else {
		printf("Instruction at MEM_FU_STAGE---> EMPTY\n");
	}
	// if (ENABLE_DEBUG_MESSAGES) {
	// 	fprintf(stderr, "MEM");
	// 	print_stage_content("Instruction at MEM_FU_STAGE--->", NULL, 1, cpu, NULL, MEM, lsq_entry);
	// }
	return 0;
}

int instruction_retirement(APEX_CPU *cpu) {
	if(cpu->rob_head == -1) {
		//Nothing to commit
		return 0;
	}
	ROB_ENTRY *rob_entry = &cpu->ROB[cpu->rob_head];
	if(rob_entry->result_valid == 1) {
		char* opcode = rob_entry->instruction_type;
		if(!(strcmp(opcode, "STORE") == 0 || strcmp(opcode, "STR") == 0 || strcmp(opcode, "BZ") == 0 || strcmp(opcode, "BNZ") == 0 || strcmp(opcode, "JUMP") == 0 || strcmp(opcode, "HALT") == 0 )) {
			cpu->regs[rob_entry->arch_register] = rob_entry->result;
			if(cpu->consumers[rob_entry->phys_register] == 0 && cpu->rename_table[rob_entry->arch_register] != rob_entry->phys_register) {
				cpu->free_PR_list[rob_entry->phys_register] = 1;
			}
		}if(strcmp(opcode, "HALT") == 0) {
			return 1;
		}if(strcmp(opcode, "BZ") == 0 || strcmp(opcode, "BNZ") == 0) {
			cpu->bis_head = (cpu->bis_head + 1) % BIS_SIZE;
			cpu->bis_current_size -= 1;
		}
		if(ENABLE_DEBUG_MESSAGES) {
			printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
			printf("Details of ROB Retired Instructions –\n");
			print_rob(cpu, rob_entry->pc_value);
			printf("\n");
			printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
		}

		int next_head = cpu->rob_head + 1;
		cpu->rob_head = next_head % ROB_SIZE;
		cpu->rob_current_size -= 1;
		free_physical_registers(cpu, -1,-1,-1);
		cpu->ins_completed += 1;
	}
	return 0;
}
int flush(APEX_CPU* cpu, BTB_ENTRY* btb_entry, IQ_ENTRY* iq_entry, int target_address) {
	cpu->pc = target_address;
	fprintf(stderr, "target address updated = %d\n", cpu->pc);
	if(strcmp(iq_entry->opcode, "BZ") == 0 || strcmp(iq_entry->opcode, "BNZ") == 0) {
		int bis_index = iq_entry->bis_index;
		int second_bis_index = -2;
		if(bis_index != cpu->bis_tail) {
			second_bis_index = cpu->bis_tail;
		}
		int i;
		for(i = 0; i < IQ_SIZE; i++) {
			IQ_ENTRY* entry = &(cpu->IQ[i]);
			if(entry->bis_index == bis_index || entry->bis_index == second_bis_index) {
				cpu->iq_free[i] = 1;
			}
		}
		if(cpu->lsq_current_size > 0) {
			int counter = cpu->lsq_current_size;
			for(i = cpu->lsq_head; counter > 0; i = (i+1)%LSQ_SIZE) {
				LSQ_ENTRY* entry = &(cpu->LSQ[i]);
				if(entry->bis_index == bis_index || entry->bis_index == second_bis_index) {
					cpu->lsq_head++;
					cpu->lsq_current_size -= 1;
				}
				counter -= 1;
			}
		}
		for(i = INT1; i <= MUL3; i++) {
			CPU_Stage* stage = &cpu->stage[i];
			IQ_ENTRY* iq_entry = &stage->iq_entry;
			if(iq_entry->bis_index == bis_index || iq_entry->bis_index == second_bis_index) { 
				strcpy(stage->opcode, "NOP");
				iq_entry->stage_finished = NUM_STAGES;
			}
		}
		int no_of_flushed_ins = 0;
		int rob_index_of_branch = (&cpu->BIS[iq_entry->bis_index])->rob_index;
		while(rob_index_of_branch != cpu->rob_tail) {
			no_of_flushed_ins += 1;
			rob_index_of_branch = (rob_index_of_branch + 1)%ROB_SIZE;
		}
		cpu->rob_tail = (&cpu->BIS[iq_entry->bis_index])->rob_index;
		cpu->rob_current_size -= no_of_flushed_ins;
		cpu->bis_tail = iq_entry->bis_index;
		for(i = 0; i < ARF_SIZE; i++) {
			cpu->rename_table[i] = (&cpu->BIS[iq_entry->bis_index])->checkpoint_entry == 1 ? cpu->checkpoint_rename_table_1[i] : cpu->checkpoint_rename_table_2[i];
		}
		for(i = 0; i < PRF_SIZE; i++) {
			cpu->free_PR_list[i] = (&cpu->BIS[iq_entry->bis_index])->checkpoint_entry == 1 ? cpu->free_PR_list_checkpoint_1[i] : cpu->free_PR_list_checkpoint_2[i]; 
		}
	}
	
	CPU_Stage* fetch_stage = &cpu->stage[F];
	strcpy(fetch_stage->opcode, "NOP");
	CPU_Stage* decode_stage = &cpu->stage[DRF];
	strcpy(decode_stage->opcode, "NOP");
	(&cpu->stage[F])->stalled = (&cpu->stage[DRF])->stalled = 0;
	//For clearing, if instruction is JUMP just compare the pc value and remove everything which has a greater value.
	//Remove IQ entries using bis_index of the branch iq_entry - whichever is having the same bis_index or the later ones
	//Same for LSQ entries
	//Clear all stages with bis index from branch and later
	//Clear fetch and decode
	//Clear ROB
	//Clear BIS
	//Copy checkpoint rename table values to rename table
	//Copy free checkpoint array back to free list
	//Free up the checkpoint table used.
	//Handle jump
	return 0;
}
int print_register_state(APEX_CPU* cpu) {
  printf("=============== STATE OF ARCHITECTURAL REGISTER FILE ==========\n");
  int index;
  int no_registers = (int) (sizeof(cpu->regs)/sizeof(cpu->regs[0]));
  for(index = 0; index < no_registers - 1; ++index) {//Assummin CC register is also part of the register file
    printf("| \t REG[%d] \t | \t Value=%d \t | \t STATUS=%s \t |\n", index, cpu->regs[index], (cpu->regs_valid[index] ? "VALID" : "INVALID"));
  }
  return 0;
}

int print_data_memory(APEX_CPU* cpu) {
  printf("============== STATE OF DATA MEMORY =============\n");
  int index;
  for(index = 0; index < 4000; ++index) {
	  if(cpu->data_memory[index] != 0) {
    		printf("| \t MEM[%d] \t | \t Data Value=%d \t |\n", index, cpu->data_memory[index]);
	  }
  }
  return 0;
}
/*
 *  APEX CPU simulation loop
 *
 *  Note : You are free to edit this function according to your
 * 				 implementation
 */
int APEX_cpu_run(APEX_CPU *cpu, int no_of_cycles, int flag)
{
	ENABLE_DEBUG_MESSAGES = flag;
	while (cpu->clock < no_of_cycles)
	{
		if (ENABLE_DEBUG_MESSAGES)
		{
			printf("^^^^^^^^^^^^^^^^^^^^^^^^^^^^ CLOCK CYCLE %d ^^^^^^^^^^^^^^^^^^^^^^^^^^^\n", cpu->clock);
		}
		//NOTE: Need to rewrite this.
		// memory(cpu);
		//fprintf(stderr, "Test APEX 1");
		int is_halt = instruction_retirement(cpu);
		if(is_halt) {
			break;
		}
		//fprintf(stderr, "Test APEX 2");
		memory_issue(cpu);
		//fprintf(stderr, "Test APEX 3");
		branch_fu(cpu);
		//fprintf(stderr, "Test APEX 4");
		writeToLSQ(cpu);
		//fprintf(stderr, "Test APEX 5");
		mul_fu_3(cpu);
		//fprintf(stderr, "Test APEX 6");
		mul_fu_2(cpu);
		//fprintf(stderr, "Test APEX 7");
		mul_fu_1(cpu);
		//fprintf(stderr, "Test APEX 8");
		int_fu_2(cpu);
		//fprintf(stderr, "Test APEX 9");
		int_fu_1(cpu);
		//fprintf(stderr, "Test APEX 10");
		issue_queue(cpu);
		//fprintf(stderr, "Test APEX 11");
		decode(cpu);
		//fprintf(stderr, "Test APEX 12\n");
		fetch(cpu);
		if(flush_and_reload) {
			flush_and_reload = 0;
			flush(cpu, mispredicted_branch_btb_entry, mispredicted_branch_iq_entry, mispredicted_branch_target_address_new);
		}
		cpu->clock++;
	}
	printf("(apex) >> Simulation Complete\n");
  	print_register_state(cpu);
  	print_data_memory(cpu);
	return 0;
}
