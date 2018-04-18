/* 046267 Computer Architecture - Spring 2017 - HW #1 */
/* This file should hold your implementation of the CPU pipeline core simulator */

#include "sim_api.h"
#define NROW 2
#define NCOL 2
#define NOVALID -1
#define EXES 0
#define MEMS 1

//***************************
// the struct is used to transfer values between the stages and for Forwarding
typedef struct {
	//int exe_index;
	int exe_value;
	//int mem_idex;
	int mem_value;
	//int wb_index;
	int wb_value;
} dstValStruct;


typedef struct {
	bool noForward;bool valid1;bool valid2;bool validDst;
	int32_t val1;
	int32_t val2;
	int32_t valDst;
} forwardUnit;

//************************** Function
void decodeStage(bool &isStall, int* pDstValDec);
void executeStage();
void memoryStage(bool &isStall, bool &isBranch);
void wbStage();
void initFU(forwardUnit* pForUn);
bool checkForwarding(SIM_cmd *pCurCmd, forwardUnit& forUn);
void initNopState(PipeStageState *nopState);

//************************** Global Variables
dstValStruct *dstVal;
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
		dstVal = new int[NROW][NCOL];
	} catch (std::bad_alloc) {
		return -1;
	}
	dstVal = {{0,0} {0,0}};
	prevState->pc = 0;

	for (int i = 0; i < SIM_PIPELINE_DEPTH; i++) {
		prevState->pipeStageState[i].cmd = CMD_NOP;
		prevState->pipeStageState[i].src1Val = 0;
		prevState->pipeStageState[i].src2Val = 0;
	}

	for (int i = 0; i < SIM_REGFILE_SIZE; i++) {
		prevState->regFile[i] = 0;
	}

	pcState[DECODE] = 0;
	pcState[FETCH] = 0;
	//for (int i=0 ; i<SIM_PIPELINE_DEPTH ; i++){
	//	pcState[i] = 0;
	//}
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
	int dstValDec[1][1] = { NOVALID, 0 }; //{isValid , value} the dst value for BRANCH, STORE commands
	int nextDstValExe[1][1] = { NOVALID, 0 }; //the dst value for the next cycle

	//fetch stage
	SIM_MemInstRead(prevState->pc, &newCmd);
	newState->cmd = newCmd;
	newState->src1Val = 0;
	newState->src2Val = 0;

	//decode + RF
	decodeStage(isStallDec, &dstValDec);

	//Execute
	executeStage(nextDstValExe, dstValDec);

	//Memory
	memoryStage(isStallMem, isBranch);

	//WB
	wbStage();

	//update PC and
	if (isStallMem) {
		//Stall - do not change the PC + move WB
		//do before Dec Stall because Dec Stall is included
		PipeStageState *nopState = new PipeStageState;
		initNopState(nopState);
		prevState->pipeStageState[WRITEBACK] = *nopState;
		dstVal[0][MEMS] = NOVALID;
		dstVal[1][MEMS] = 0;
	} else if (isStallDec) {
		// Stall - do not change the PC + move EXE,MEM,WB
		PipeStageState *nopState = new PipeStageState;
		initNopState(nopState);
		prevState->pipeStageState[WRITEBACK] =
				prevState->pipeStageState[MEMORY];
		prevState->pipeStageState[MEMORY] = prevState->pipeStageState[EXECUTE];
		prevState->pipeStageState[EXECUTE] = prevState->pipeStageState[DECODE];
		prevState->pipeStageState[DECODE] = *nopState;
		dstVal[0][EXES] = nextDstValExe[0];
		dstVal[1][EXES] = nextDstValExe[1];
		dstVal[0][MEMS] = dstVal[0][EXES];
		dstVal[1][MEMS] = dstVal[1][EXES];
		pcState[DECODE] = NOVALID;
	} else if (isBranch) {
		//Branch - change the PC to the new dest + FLUSH  TODO: Neta
	} else {
		//PC+4
		prevState->pc = (prevState->pc) + 4;
		prevState->pipeStageState[WRITEBACK] = prevState->pipeStageState[MEMORY]
		prevState->pipeStageState[MEMORY] = nextState[MEMORY];
		prevState->pipeStageState[EXECUTE] = nextState[EXECUTE];
		prevState->pipeStageState[DECODE] = nextState[DECODE];
		prevState->pipeStageState[FETCH] = nextState[FETCH];
		dstVal[0][EXES] = nextDstValExe[0];
		dstVal[1][EXES] = nextDstValExe[1];
		dstVal[0][MEMS] = dstVal[0][EXES];
		dstVal[1][MEMS] = dstVal[1][EXES];
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
	nopState->dst = 0;
	nopState->src1Val = 0;
	nopState->src2Val = 0;
	return;
}


/* the Decode and RF stage:
 ~ read from the RF and update src(1/2)val and dstVal for branch commands

 Flags:
 ~ isStall = RAW - check if register is needed and is rewriten in previous cmds
 */
void decodeStage(bool &isStall, int* pDstValDec) {
	PipeStageState *curStage = prevState->pipeStageState[FETCH];
	SIM_cmd *pCurCmd = &(curStage->cmd);     //TODO: check if right probably not
	forwardUnit *pForUn = new forwardUnit;
	initFU(pforUn); //intializing the Forward Unit;

	bool checkForward = checkForwarding(pCurCmd, pForUn);
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
		pDstValDec[0] = 1;
		if (pForUn->validDst)
			pDstValDec[1] = pForUn->valDst;
		else
			pDstValDec[1] = prevState->regFile[pCurCmd->dst];
	}
	return;
}

bool checkForwarding(SIM_cmd *pCurCmd, forwardUnit& forUn) {
	bool isForwardingNeeded = false;
	int reg1 = pCurCmd->src1;
	int reg2 = (pCurCmd->isSrc2Imm) ? pCurCmd->src2 : NOVALID;
	if (dstVal[0][EXES] == reg1) {
		isForwardingNeeded = true;
		if (forwarding) {
			forUn.valid1 = true;
			forUn.val1 = dstVal[1][EXES];
		}
	} else if (dstVal[0][MEMS] == reg1) {
		isForwardingNeeded = true;
		if (forwarding || split_regfile) {
			forUn.noForward = (split_regfile) ? true : false;
			forUn.valid1 = true;
			forUn.val1 = dstVal[1][MEMS];
		}
	}
	if (reg2 != NOVALID) {
		if (dstVal[0][EXES] == reg2) {
			isForwardingNeeded = true;
			if (forwarding) {
				forUn.valid2 = true;
				forUn.val2 = dstVal[1][EXES];
			}
		} else if (dstVal[0][MEMS] == reg2) {
			isForwardingNeeded = true;
			if (forwarding || split_regfile) {
				forUn.noForward = (split_regfile) ? true : false;
				forUn.valid2 = true;
				forUn.val2 = dstVal[1][MEMS];
			}
		}
	}
	if ((pCurCmd->opcode == CMD_BR) || (pCurCmd->opcode == CMD_BREQ)
			|| (pCurCmd->opcode == CMD_BRNEQ)) {
		int dst = pCurCmd->dst;
		if (dstVal[0][EXES] == dst) {
			isForwardingNeeded = true;
			if (forwarding) {
				forUn.validDst = true;
				forUn.valDst = dstVal[1][EXES];
			} else if (dstVal[0][MEMS] == dst) {
				isForwardingNeeded = true;
				if (forwarding || split_regfile) {
					forUn.noForward = (split_regfile) ? true : false;
					forUn.validDst = true;
					forUn.valDst = dstVal[1][MEMS];
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
void executeStage() {
	SIM_cmd cmd = prevState->pipeStageState[DECODE].cmd;

}


/* the Memory Stage:
 ~ Check if branch is needed
 ~ Get information from the main memory

 Flags:
 ~ isStall = stall the pipe if couldn't retrieve data from the main memory
 ~ isBranch = true if branch is taken.
 */
void memoryStage(bool &isStall, bool &isBranch) {
	//Check if branch is needed
	SIM_cmd cmd = prevState->pipeStageState[EXECUTE].cmd;
	int32_t src1Val = prevState->pipeStageState[EXECUTE].src1Val;
	int32_t src2Val = prevState->pipeStageState[EXECUTE].src2Val;

	if (cmd.opcode == CMD_BR) {
		isBranch = true;
		return;
	} else if (cmd.opcode == CMD_BREQ){
		if (src1Val == src2Val){
			isBranch = true;
		}
		return;
	} else if (cmd.opcode == CMD_BRNEQ){
		if (src1Val != src2Val){
			isBranch = true;
		}
		return;
	}

	//check if need to read something from mem: (may take more then 1 cycle)
	if (cmd.opcode == CMD_LOAD){
		//CMD_LOAD, dst <- Mem[src1 + src2]  (src2 may be an immediate)
		if (SIM_MemDataRead( , ) != 0){ //the read isn't complete
			isStall = true;
			return;
		}
	}

	//check if need to write something to mem:
	if(cmd.opcode == CMD_STORE){
		SIM_MemDataWrite( , );
		//CMD_STORE, Mem[dst + src2] <- src1  (src2 may be an immediate)
	}

	return;
}


/* the Write Back Stage:
 ~ writing the data back to the registers (ADD.ADDI.SUB,SUBI,LOAD)
 */
void wbStage() {
	if (dstVal[0][MEMS] != NOVALID){
		prevState->regFile[dstVal[0][MEMS]] = dstVal[1][MEMS];
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

