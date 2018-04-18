/* 046267 Computer Architecture - Spring 2017 - HW #1 */
/* This file should hold your implementation of the CPU pipeline core simulator */

#include "sim_api.h"
#include <iostream>
#define NOVALID -1
//***************************
// the struct is used to transfer values between the stages and for Forwarding
typedef struct {
	int exe_index;
	int mem_index;
	int dst_value;
	int exe_value;
	int mem_value;
} dataStruct;


typedef struct {
	bool noForward;bool valid1;bool valid2;bool validDst;
	int32_t val1;
	int32_t val2;
	int32_t valDst;
} forwardUnit;

//************************** Function
void decodeStage(bool &isStall, dataStruct *nextData);
void executeStage(dataStruct *nextDstVal);
void memoryStage(bool &isStall, bool &isBranch, dataStruct *nextDstVal);
void wbStage();
void initFU(forwardUnit* pForUn);
bool checkForwarding(SIM_cmd *pCurCmd, forwardUnit& forUn);
void initNopState(PipeStageState *nopState);

//************************** Global Variables
dataStruct *data;
SIM_coreState *prevState;
int32_t *pcState;

//*************************** Function Implementation

/*! SIM_CoreReset: Reset the processor core simulator machine to start new simulation
 Use this API to initialize the processor core simulator's data structures.
 The simulator machine must complete this call with these requirements met:
 - PC = 0  (entry point for a program is at address 0)
 - All the register file is cleared (all registers hold 0)
 - The value of IF is the instuction in address 0x0
 \returns 0 on success. <0 in case of initialization failure.
 */
int SIM_CoreReset(void) {
	try {
		prevState = new SIM_coreState;
		pcState = new int[SIM_PIPELINE_DEPTH - 3]; //needed only until the EXECUTE stage
		data = new dataStruct;
	} catch (std::bad_alloc& e) {
		return -1;
	}
	data->exe_value = 0;
  data->mem_value = 0;
  data->dst_value = 0;
  data-> exe_index = NOVALID;
  data->mem_index = NOVALID;
  prevState->pc = 0;

	for (int i = 0; i < SIM_PIPELINE_DEPTH; i++) {
		prevState->pipeStageState[i].cmd.opcode = CMD_NOP;
		prevState->pipeStageState[i].src1Val = 0;
		prevState->pipeStageState[i].src2Val = 0;
	}

	for (int i = 0; i < SIM_REGFILE_SIZE; i++) {
		prevState->regFile[i] = 0;
	}

	pcState[DECODE] = 0;
	pcState[FETCH] = 0;
	return 0;
}

/*! SIM_CoreClkTick: Update the core simulator's state given one clock cycle.
 This function is expected to update the core pipeline given a clock cycle event.
 */
void SIM_CoreClkTick() {
	PipeStageState *newState = new PipeStageState; // needed only
	bool isStallDec = false;
	bool isStallMem = false;
	bool isBranch = false;
	SIM_cmd newCmd;

	dataStruct *nextData = new dataStruct;

	//fetch stage
	SIM_MemInstRead(prevState->pc, &newCmd);
	newState->cmd = newCmd;
	newState->src1Val = 0;
	newState->src2Val = 0;

	//decode + RF
	decodeStage(isStallDec, nextData);

	//Execute
	executeStage(nextData);

	//Memory
	memoryStage(isStallMem, isBranch, nextData);

	//WB
	wbStage();

	//update PC and
	if (isStallMem) {
		//Stall - do not change the PC + move WB
		//do before Dec Stall because Dec Stall is included
		PipeStageState *nopState = new PipeStageState;
		initNopState(nopState);
    prevState->pipeStageState[WRITEBACK]= prevState->pipeStageState[MEMORY];
		prevState->pipeStageState[MEMORY] = *nopState;
    data->mem_index = NOVALID;
    data->mem_value = 0 ;
	} else if (isStallDec) {
		// Stall - do not change the PC + move EXE,MEM,WB
		PipeStageState *nopState = new PipeStageState;
		initNopState(nopState);
		prevState->pipeStageState[WRITEBACK] = prevState->pipeStageState[MEMORY];
		prevState->pipeStageState[MEMORY] = prevState->pipeStageState[EXECUTE];
		prevState->pipeStageState[EXECUTE] = prevState->pipeStageState[DECODE];
		prevState->pipeStageState[DECODE] = *nopState;
		data = nextData;
    pcState[DECODE] = NOVALID;
	} else if (isBranch) {
		//Branch - change the PC to the new dest + FLUSH  TODO: Neta
	} else {
		//PC+4
		prevState->pc = (prevState->pc) + 4;
		prevState->pipeStageState[WRITEBACK] = prevState->pipeStageState[MEMORY];
    prevState->pipeStageState[MEMORY] = prevState->pipeStageState[EXECUTE];
		prevState->pipeStageState[EXECUTE] = prevState->pipeStageState[DECODE];
		prevState->pipeStageState[DECODE] = prevState->pipeStageState[FETCH];
		prevState->pipeStageState[FETCH] = *newState;
    data = nextData;
		pcState[DECODE] = pcState[FETCH];
		pcState[FETCH] = prevState->pc;
	}
	return;
}

void initNopState(PipeStageState *nopState) {
	nopState->cmd.opcode = CMD_NOP;
	nopState->cmd.src1 = 0;
	nopState->cmd.src2 = 0;
	nopState->cmd.isSrc2Imm = false;
	nopState->cmd.dst = 0;
	nopState->src1Val = 0;
	nopState->src2Val = 0;
	return;
}


/* the Decode and RF stage:
 ~ read from the RF and update src(1/2)val and dstVal for branch commands

 Flags:
 ~ isStall = RAW - check if register is needed and is rewriten in previous cmds
 */
 void decodeStage(bool &isStall, dataStruct *nextData) {
	PipeStageState *curStage = &prevState->pipeStageState[FETCH];
	SIM_cmd *pCurCmd = &(curStage->cmd);     //TODO: check if right probably not
	forwardUnit *pForUn = new forwardUnit;
	initFU(pForUn); //intializing the Forward Unit;

	bool checkForward = checkForwarding(pCurCmd, *pForUn);
	if ((!forwarding) && (checkForward) && (!pForUn->noForward)) {
		isStall = true;
		return;
	}

	//src1Val and src2Val are needed for all command except BR , NOP and HALT
	if ((pCurCmd->opcode != CMD_BR) || (pCurCmd->opcode != CMD_HALT)
			|| (pCurCmd->opcode != CMD_NOP)) {
		if (pForUn->valid1) //retrieving src1Val
			curStage->src1Val = pForUn->val1;
		else {
			curStage->src1Val = prevState->regFile[pCurCmd->src1];
		}
		if (!pCurCmd->isSrc2Imm) { //retrieving src2Val if not immediate
			if (pForUn->valid2)
				curStage->src2Val = pForUn->val2;
			else
				curStage->src2Val = prevState->regFile[pCurCmd->src2];
		} else {
			curStage->src2Val = pCurCmd->src2;
		}
	}

	// dstVal is needed only for the BRANCH commands
	if ((pCurCmd->opcode == CMD_BR) || (pCurCmd->opcode == CMD_BREQ)
			|| (pCurCmd->opcode == CMD_BRNEQ)) { //retrieving dst Value
		if (pForUn->validDst)
			nextData->dst_value = pForUn->valDst;
		else
			nextData->dst_value = prevState->regFile[pCurCmd->dst];
	}
  else
    nextData->dst_value = NOVALID;
	return;
 }

bool checkForwarding(SIM_cmd *pCurCmd, forwardUnit& forUn) {
	bool isForwardingNeeded = false;
	int reg1 = pCurCmd->src1;
	int reg2 = (pCurCmd->isSrc2Imm) ? pCurCmd->src2 : NOVALID;
	if (data->exe_index == reg1) {
		isForwardingNeeded = true;
		if (forwarding) {
			forUn.valid1 = true;
			forUn.val1 = data->exe_value;
		}
	} else if (data->mem_index == reg1) {
		isForwardingNeeded = true;
		if (forwarding || split_regfile) {
			forUn.noForward = (split_regfile) ? true : false;
			forUn.valid1 = true;
			forUn.val1 = data->mem_value;
		}
	}
	if (reg2 != NOVALID) {
		if (data->exe_index == reg2) {
			isForwardingNeeded = true;
			if (forwarding) {
				forUn.valid2 = true;
				forUn.val2 = data->exe_value;
			}
		} else if (data->mem_index == reg2) {
			isForwardingNeeded = true;
			if (forwarding || split_regfile) {
				forUn.noForward = (split_regfile) ? true : false;
				forUn.valid2 = true;
				forUn.val2 = data->mem_value;
			}
		}
	}
	if ((pCurCmd->opcode == CMD_BR) || (pCurCmd->opcode == CMD_BREQ)
			|| (pCurCmd->opcode == CMD_BRNEQ)) {
		int dst = pCurCmd->dst;
		if (data->exe_index == dst) {
			isForwardingNeeded = true;
			if (forwarding) {
				forUn.validDst = true;
				forUn.valDst = data->exe_value;
			} else if (data->mem_index == dst) {
				isForwardingNeeded = true;
				if (forwarding || split_regfile) {
					forUn.noForward = (split_regfile) ? true : false;
					forUn.validDst = true;
					forUn.valDst = data->mem_index;
				}
			}
		}
		return isForwardingNeeded;
	}
}

void initFU(forwardUnit* pForUn) {
	pForUn->noForward = false;
	pForUn->valid1 = false;
	pForUn->valid2 = false;
	pForUn->validDst = false;
	pForUn->val1 = 0;
	pForUn->val2 = 0;
	pForUn->valDst = 0;
}


/* The Execute stage:
 ~ Arithmetic operations (ADD,SUB,ADDI,SUBI)
 ~ Comparison operations (BREQ,BRNEQ)
 ~ Branch Destination Calculations (BR,BREQ,BRNEQ)
 ~ Memory Destination Calculations (LOAD,STORE)
 */
void executeStage(dataStruct *nextData) {
	SIM_cmd_opcode cmd = prevState->pipeStageState[DECODE].cmd.opcode;
	int32_t src1Val = prevState->pipeStageState[EXECUTE].src1Val;
	int32_t src2Val = prevState->pipeStageState[EXECUTE].src2Val;

	switch (cmd){
	case CMD_ADD: nextData->exe_value = src1Val + src2Val; break;
	case CMD_SUB: nextData->exe_value = src1Val - src2Val; break;
	case CMD_ADDI: nextData->exe_value = src1Val + src2Val; break;
	case CMD_SUBI: nextData->exe_value = src1Val - src2Val; break;
	case CMD_LOAD: nextData->exe_value = src1Val + src2Val; break;
	case CMD_STORE: nextData->exe_value = data->dst_value + src2Val; break;
	case CMD_BR: nextData->exe_value = data->dst_value + pcState[DECODE]; break;
	case CMD_BREQ: nextData->exe_value = data->dst_value + pcState[DECODE]; break;
	case CMD_BRNEQ: nextData->exe_value = data->dst_value + pcState[DECODE]; break;
	case CMD_HALT: nextData->exe_value = 0; break;
	}

}


/* the Memory Stage:
 ~ Check if branch is needed
 ~ Get information from the main memory

 Flags:
 ~ isStall = stall the pipe if couldn't retrieve data from the main memory
 ~ isBranch = true if branch is taken.
 */
void memoryStage(bool &isStall, bool &isBranch, dataStruct *nextData) {
	//Check if branch is needed
	SIM_cmd_opcode cmd = prevState->pipeStageState[EXECUTE].cmd.opcode;
	int32_t src1Val = prevState->pipeStageState[EXECUTE].src1Val;
	int32_t src2Val = prevState->pipeStageState[EXECUTE].src2Val;

	nextData->mem_value = data->exe_value;

	if (cmd == CMD_BR) {
		isBranch = true;
		return;
	} else if (cmd == CMD_BREQ){
		if (src1Val == src2Val){
			isBranch = true;
		}
		return;
	} else if (cmd == CMD_BRNEQ){
		if (src1Val != src2Val){
			isBranch = true;
		}
		return;
	}

	if (cmd == CMD_LOAD){
		if (SIM_MemDataRead(data->exe_value, &nextData->mem_value) != 0){ //the read isn't complete
			nextData->mem_value = 0;
			isStall = true;
			return;
		}
	}

	if(cmd == CMD_STORE){
		SIM_MemDataWrite(data->exe_value, src1Val);
	}

	return;
}


/* the Write Back Stage:
 ~ writing the data back to the registers (ADD.ADDI.SUB,SUBI,LOAD)
 */
void wbStage() {
	//if (dstVal[0][MEMS] != NOVALID){
	//	prevState->regFile[dstVal[0][MEMS]] = dstVal[1][MEMS];
	//}

	SIM_cmd_opcode cmd = prevState->pipeStageState[MEMORY].cmd.opcode;

	if (cmd == CMD_ADD || cmd == CMD_SUB || cmd == CMD_ADDI || cmd == CMD_SUBI
			|| cmd == CMD_LOAD){
		int dst_reg = prevState->pipeStageState[MEMORY].cmd.dst;
		prevState->regFile[dst_reg] = data->mem_value;
	}

	return;
}


/*! SIM_CoreGetState: Return the current core (pipeline) internal state
 prevState: The returned current pipeline state
 The function will return the state of the pipe at the end of a cycle
 */
void SIM_CoreGetState(SIM_coreState *curState) {
	curState = prevState;
}

