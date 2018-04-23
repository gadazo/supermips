/* 046267 Computer Architecture - Spring 2017 - HW #1 */
/* This file should hold your implementation of the CPU pipeline core simulator */

#include "sim_api.h"
#include <iostream>
#define NOVALID -1
//***************************
// the struct is used to transfer values between the stages and for Forwardin

struct dataStruct {
	int exe_index;
	int mem_index;
	int dst_value;
	int exe_value;
	int mem_value;
};

typedef struct {
	bool noForward;
	bool valid1;
	bool valid2;
	bool validDst;
	int32_t val1;
	int32_t val2;
	int32_t valDst;
} forwardUnit;

//************************** Function
void decodeStage(bool &isStall, dataStruct &nextData);
void executeStage(dataStruct &nextData);
void memoryStage(bool &isStall, bool &isBranch, dataStruct &nextData);
void wbStage();
void initFU(forwardUnit* pForUn);
bool checkForwarding(SIM_cmd *pCurCmd, forwardUnit& forUn);
void initNopState(PipeStageState *nopState);
void initData(dataStruct &dataReg);

//************************** Global Variables
dataStruct *dataReg;
SIM_coreState *prevState;
SIM_coreState *nextRegisters;
int32_t *pcState;
bool prevStallDec ;
bool prevStallMem ;
bool prevBranch ;

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
		dataReg = new dataStruct;
	} catch (std::bad_alloc& e) {
		return -1;
	}

	dataReg->dst_value = 0;
	dataReg->exe_value = 0;
	dataReg->mem_value = 0;
	dataReg->exe_index = NOVALID;
	dataReg->mem_index = NOVALID;
  prevStallDec = false;
  prevStallMem = false;
  prevBranch =false;

	prevState->pc = 0;

	for (int i = 0; i < SIM_PIPELINE_DEPTH; i++) {
		prevState->pipeStageState[i].cmd.opcode = CMD_NOP;
		prevState->pipeStageState[i].cmd.src1 = 0;
		prevState->pipeStageState[i].cmd.src2 = 0;
		prevState->pipeStageState[i].cmd.isSrc2Imm = false;
		prevState->pipeStageState[i].cmd.dst = 0;
		prevState->pipeStageState[i].src1Val = 0;
		prevState->pipeStageState[i].src2Val = 0;
	}

	for (int i = 0; i < SIM_REGFILE_SIZE; i++) {
		prevState->regFile[i] = 0;
	}


	nextRegisters = new SIM_coreState;
	for (int i = 0; i < SIM_REGFILE_SIZE; i++) {
		nextRegisters->regFile[i] = 0;
	}

	SIM_cmd newCmd;
	SIM_MemInstRead(prevState->pc, &newCmd);
	prevState->pipeStageState[FETCH].cmd = newCmd;
	prevState->pipeStageState[FETCH].src1Val = 0;
	prevState->pipeStageState[FETCH].src2Val = 0;
	pcState[FETCH] = 0;
	pcState[DECODE] = 0;
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

	dataStruct nextData;
	initData(nextData);
  // std::cout<<"DEBUG : "<<"mem_value : "<<dataReg->mem_value<<" mem_index : "<<dataReg->mem_index<<std::endl;
  // std::cout<<"DEBUG : "<<"exe_value : "<<dataReg->exe_value<<" exe_index : "<<dataReg->exe_index<<std::endl;
 try{
   //update PC and FETCH
   if (prevStallMem) {
     //Stall - do not change the PC + move WB
     //do before Dec Stall because Dec Stall is included
     PipeStageState *nopState = new PipeStageState;
     initNopState(nopState);
     prevState->pipeStageState[WRITEBACK] =*nopState;
     dataReg->mem_index = NOVALID;
     dataReg->mem_value = 0;
     prevState->pc = nextRegisters->pc;
   }
   else if (prevBranch) {
     //Branch - change the PC to the new dest + FLUSH
     prevState->pc = dataReg->mem_value;
     // std::cout<<"DEBUG : "<<"new pc: "<<dataReg->mem_value<<std::endl;
     PipeStageState *nopState = new PipeStageState;
     initNopState(nopState);
     SIM_MemInstRead(prevState->pc, &newCmd);
     newState->cmd = newCmd;
     // std::cout<<"DEBUG : "<<"CMD_OPCODE : "<<newCmd.opcode<<std::endl;
     newState->src1Val = 0;
     newState->src2Val = 0;
     prevState->pipeStageState[FETCH] = *newState;
     prevState->pipeStageState[WRITEBACK] =
       prevState->pipeStageState[MEMORY];
     prevState->pipeStageState[MEMORY] = *nopState;
     prevState->pipeStageState[DECODE] = *nopState;
     prevState->pipeStageState[EXECUTE] = *nopState;
     dataReg->dst_value = 0;
     dataReg->exe_value = 0;
   }
   else if (prevStallDec) {
     // Stall - do not change the PC + move EXE,MEM,WB
     PipeStageState *nopState = new PipeStageState;
     initNopState(nopState);
     prevState->pipeStageState[WRITEBACK] =
       prevState->pipeStageState[MEMORY];
     prevState->pipeStageState[MEMORY] = prevState->pipeStageState[EXECUTE];
     prevState->pipeStageState[EXECUTE] = *nopState;
     pcState[DECODE] = NOVALID;
     prevState->pc = nextRegisters->pc;
   } else {
     //PC+4
     SIM_cmd_opcode CurOpcode = prevState->pipeStageState[FETCH].cmd.opcode;
     bool isImm = prevState->pipeStageState[FETCH].cmd.isSrc2Imm;
     nextRegisters->pc = (prevState->pc) + 4;
     prevState->pipeStageState[WRITEBACK] =
       prevState->pipeStageState[MEMORY];
     prevState->pipeStageState[MEMORY] = prevState->pipeStageState[EXECUTE];
     prevState->pipeStageState[EXECUTE] = prevState->pipeStageState[DECODE];
     if (((CurOpcode == CMD_ADDI) || (CurOpcode == CMD_SUBI) || (CurOpcode == CMD_LOAD) ||(CurOpcode == CMD_STORE) ) && (isImm)){
       prevState->pipeStageState[FETCH].src2Val = prevState->pipeStageState[FETCH].cmd.src2;
     }     prevState->pipeStageState[DECODE] = prevState->pipeStageState[FETCH];
     prevState->pc = nextRegisters->pc;
     pcState[DECODE] = pcState[FETCH];
     pcState[FETCH] = prevState->pc;
     SIM_MemInstRead(nextRegisters->pc, &newCmd);
     newState->cmd = newCmd;
     newState->src1Val = 0;
     newState->src2Val = 0;
     prevState->pipeStageState[FETCH] = *newState;
   }
 }
 catch(std::bad_alloc& e){
    std::cout<<"Out of Memory"<<std::endl;
 }

 if (!split_regfile && !forwarding){
   for (int i = 0; i < SIM_REGFILE_SIZE; i++) {
     prevState->regFile[i] = nextRegisters->regFile[i];
   }
 }

 wbStage();
 decodeStage(isStallDec, nextData);
 executeStage(nextData);
 memoryStage(isStallMem, isBranch, nextData);

 prevBranch=isBranch;
  prevStallDec = isStallDec;
  prevStallMem = isStallMem;
  if(!isStallMem)
    *dataReg = nextData;
  return;
}

void initData(dataStruct &dataReg){
	dataReg.dst_value = 0;
	dataReg.exe_index = NOVALID;
	dataReg.exe_value = 0;
	dataReg.mem_index = NOVALID;
	dataReg.mem_value = 0;
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
void decodeStage(bool &isStall, dataStruct &nextData) {
	PipeStageState *curStage = &prevState->pipeStageState[DECODE];
	SIM_cmd *pCurCmd = &(curStage->cmd);
	forwardUnit *pForUn = new forwardUnit;
	initFU(pForUn); //intializing the Forward Unit;

	bool checkForward = checkForwarding(pCurCmd, *pForUn);
	if ((!forwarding) && (checkForward) && (!pForUn->noForward)) {
		isStall = true;
	}


	//src1Val and src2Val are needed for all command except BR , NOP and HALT
	if ((pCurCmd->opcode != CMD_BR) || (pCurCmd->opcode != CMD_HALT)
			|| (pCurCmd->opcode != CMD_NOP)) {
		if ((pForUn->valid1) && !isStall ){//retrieving src1Val
			curStage->src1Val = pForUn->val1;
    }
    else if(!pForUn->valid1){
      curStage->src1Val = prevState->regFile[pCurCmd->src1];
    }
    if (!pCurCmd->isSrc2Imm) { //retrieving src2Val if not immediate
      if ((pForUn->valid2) && !isStall){
        curStage->src2Val = pForUn->val2;
      }
      else if(!pForUn->valid2){
          curStage->src2Val = prevState->regFile[pCurCmd->src2];
      }
    }
    else {
      curStage->src2Val = pCurCmd->src2;
    }
  }
  
// dstVal is needed only for the BRANCH commands
	if ((pCurCmd->opcode == CMD_BR) || (pCurCmd->opcode == CMD_BREQ)
			|| (pCurCmd->opcode == CMD_BRNEQ)) { //retrieving dst Value
		if ((pForUn->validDst) && !isStall){
      nextData.dst_value = pForUn->valDst;
    }
		else if(!pForUn->validDst){
      nextData.dst_value = prevState->regFile[pCurCmd->dst];
    }
	}
  else
		nextData.dst_value = NOVALID;
return;
}

bool checkForwarding(SIM_cmd *pCurCmd, forwardUnit& forUn) {
	bool isForwardingNeeded = false;
	int reg1 = pCurCmd->src1;
	int reg2 = (pCurCmd->isSrc2Imm) ? NOVALID : pCurCmd->src2;
	if ((dataReg->exe_index == reg1) && (reg1 != 0)) {
    isForwardingNeeded = true;
		if (forwarding) {
			forUn.valid1 = true;
			forUn.val1 = dataReg->exe_value;
		}
	} else if ((dataReg->mem_index == reg1) && (reg1 != 0)){
		isForwardingNeeded = true;
		if (forwarding || split_regfile) {
			forUn.noForward = (split_regfile) ? true : false;
			forUn.valid1 = true;
			forUn.val1 = dataReg->mem_value;
		}
	}
	if ((reg2 != NOVALID) && (reg2 != 0)) {
		if (dataReg->exe_index == reg2) {
			isForwardingNeeded = true;
			if (forwarding) {
				forUn.valid2 = true;
				forUn.val2 = dataReg->exe_value;
			}
		} else if (dataReg->mem_index == reg2) {
			isForwardingNeeded = true;
			if (forwarding || split_regfile) {
				forUn.noForward = (split_regfile) ? true : false;
				forUn.valid2 = true;
				forUn.val2 = dataReg->mem_value;
			}
		}
	}
	if ((pCurCmd->opcode == CMD_BR) || (pCurCmd->opcode == CMD_BREQ)
			|| (pCurCmd->opcode == CMD_BRNEQ)) {
		int dst = pCurCmd->dst;
		if ((dataReg->exe_index == dst) && (dst != 0)) {
			isForwardingNeeded = true;
			if (forwarding) {
				forUn.validDst = true;
				forUn.valDst = dataReg->exe_value;
			} else if ((dataReg->mem_index == dst)&& (dst != 0)) {
				isForwardingNeeded = true;
				if (forwarding || split_regfile) {
					forUn.noForward = (split_regfile) ? true : false;
					forUn.validDst = true;
					forUn.valDst = dataReg->mem_index;
				}
			}
		}
	}
	return isForwardingNeeded;
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
void executeStage(dataStruct &nextData) {
	SIM_cmd_opcode cmd = prevState->pipeStageState[EXECUTE].cmd.opcode;
	int32_t src1Val = prevState->pipeStageState[EXECUTE].src1Val;
	int32_t src2Val = prevState->pipeStageState[EXECUTE].src2Val;

	switch (cmd) {
	case CMD_ADD:
		nextData.exe_value = src1Val + src2Val;
    nextData.exe_index = prevState->pipeStageState[EXECUTE].cmd.dst;
		break;
	case CMD_SUB:
		nextData.exe_value = src1Val - src2Val;
    nextData.exe_index = prevState->pipeStageState[EXECUTE].cmd.dst;
    break;
	case CMD_ADDI:
		nextData.exe_value = src1Val + src2Val;
    nextData.exe_index = prevState->pipeStageState[EXECUTE].cmd.dst;
		break;
	case CMD_SUBI:
		nextData.exe_value = src1Val - src2Val;
    nextData.exe_index = prevState->pipeStageState[EXECUTE].cmd.dst;
		break;
	case CMD_LOAD:
		nextData.exe_value = src1Val + src2Val;
    nextData.exe_index = NOVALID;
		break;
	case CMD_STORE:
		nextData.exe_value = dataReg->dst_value + src2Val;
    nextData.exe_index = NOVALID;
		break;
	case CMD_BR:
		nextData.exe_value = dataReg->dst_value + pcState[DECODE];
    nextData.exe_index = NOVALID;
		break;
	case CMD_BREQ:
		nextData.exe_value = dataReg->dst_value + pcState[DECODE];
    nextData.exe_index = NOVALID;
		break;
	case CMD_BRNEQ:
		nextData.exe_value = dataReg->dst_value + pcState[DECODE];
    nextData.exe_index = NOVALID;
		break;
	case CMD_HALT:
		nextData.exe_value = 0;
    nextData.exe_index = NOVALID;
		break;
	}


}

/* the Memory Stage:
 ~ Check if branch is needed
 ~ Get information from the main memory

 Flags:
 ~ isStall = stall the pipe if couldn't retrieve data from the main memory
 ~ isBranch = true if branch is taken.
 */
void memoryStage(bool &isStall, bool &isBranch, dataStruct &nextData) {
	//Check if branch is needed
	SIM_cmd_opcode cmd = prevState->pipeStageState[MEMORY].cmd.opcode;
	int32_t src1Val = prevState->pipeStageState[MEMORY].src1Val;
	int32_t src2Val = prevState->pipeStageState[MEMORY].src2Val;
  nextData.mem_index = dataReg->exe_index;
	nextData.mem_value = dataReg->exe_value;
	if (cmd == CMD_BR) {
		isBranch = true;
		return;
	} else if (cmd == CMD_BREQ) {
		if (src1Val == src2Val) {
			isBranch = true;
		}
		return;
	} else if (cmd == CMD_BRNEQ) {
		if (src1Val != src2Val) {
			isBranch = true;
		}
		return;
	}

	if (cmd == CMD_LOAD) {
		if (SIM_MemDataRead(dataReg->exe_value, &nextData.mem_value) != 0) { //the read isn't complete
			nextData.mem_value = 0;
			isStall = true;
			return;
		}
    else{
        nextData.mem_index = prevState->pipeStageState[MEMORY].cmd.dst;
    }
	}

	if (cmd == CMD_STORE) {
		SIM_MemDataWrite(dataReg->exe_value, src1Val);
	}

	return;
}

/* the Write Back Stage:
 ~ writing the data back to the registers (ADD.ADDI.SUB,SUBI,LOAD)
 */
void wbStage() {
	SIM_cmd_opcode cmd = prevState->pipeStageState[WRITEBACK].cmd.opcode;

	if (cmd == CMD_ADD || cmd == CMD_SUB || cmd == CMD_ADDI || cmd == CMD_SUBI
			|| cmd == CMD_LOAD) {
		int dst_reg = prevState->pipeStageState[WRITEBACK].cmd.dst;
		if (!split_regfile && !forwarding){
			nextRegisters->regFile[dst_reg] = dataReg->mem_value;
		} else {
			prevState->regFile[dst_reg] = dataReg->mem_value;
		}
		//prevState->regFile[dst_reg] = data->mem_value;
		//nextRegisters->regFile[dst_reg] = data->mem_value;
	}

	return;
}

/*! SIM_CoreGetState: Return the current core (pipeline) internal state
 prevState: The returned current pipeline state
 The function will return the state of the pipe at the end of a cycle
 */
void SIM_CoreGetState(SIM_coreState *curState) {
	*curState = *prevState;
}

