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
struct forwardingUnit {
  bool src1_valid;
  int src1_value;
  bool src2_valid;
  int src2_value;
};

//************************** Function
void decodeStage(bool &isStall, dataStruct &nextData);
void executeStage(dataStruct &nextData);
void memoryStage(bool &isStall, bool &isBranch, dataStruct &nextData);
void wbStage(dataStruct& nextData);
void initNopState(PipeStageState *nopState);
void initData(dataStruct &dataReg);
void forwardUnit (SIM_cmd* pCurCmd ,int &dstIndex ,int32_t& dstValue ,bool& dstValid , bool &isStall , int curMemDst , int curExeDst );
//************************** Global Variables
dataStruct *dataReg;
SIM_coreState *prevState;
SIM_coreState *nextRegisters;
int32_t *pcState;
bool prevStallDec ;
bool prevStallMem ;
bool prevBranch ;
bool memForward;
bool wbForward;
forwardingUnit *fU;

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
    fU = new forwardingUnit[2];
	} catch (std::bad_alloc& e) {
		return -1;
	}

  fU->src1_valid = false;
  fU->src1_value = 0;
  fU->src2_valid = false;
  fU->src2_value = 0;
	dataReg->dst_value = 0;
	dataReg->exe_value = 0;
	dataReg->mem_value = 0;
	dataReg->exe_index = NOVALID;
	dataReg->mem_index = NOVALID;
  prevStallDec = false;
  prevStallMem = false;
  prevBranch =false;
  wbForward = false;
  memForward = false;

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
	PipeStageState *newState = new PipeStageState; 
	bool isStallDec = false;
	bool isStallMem = false;
	bool isBranch = false;
	SIM_cmd newCmd;

	dataStruct nextData;
	initData(nextData);
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
      PipeStageState *nopState = new PipeStageState;
      initNopState(nopState);
      SIM_MemInstRead(prevState->pc, &newCmd);
      newState->cmd = newCmd;
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
      fU[1].src1_valid =false;
      fU[1].src1_valid = 0;
      fU[1].src2_value =0;
      fU[1].src2_valid =false;
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
      fU[1].src1_valid =false;
      fU[1].src1_valid = 0;
      fU[1].src2_value =0;
      fU[1].src2_valid =false;
    }
    else {
      //PC+4
      SIM_cmd_opcode CurOpcode = prevState->pipeStageState[FETCH].cmd.opcode;
      bool isImm = prevState->pipeStageState[FETCH].cmd.isSrc2Imm;
      nextRegisters->pc = (prevState->pc) + 4;
      prevState->pipeStageState[WRITEBACK] =
        prevState->pipeStageState[MEMORY];
      prevState->pipeStageState[MEMORY] = prevState->pipeStageState[EXECUTE];
      prevState->pipeStageState[EXECUTE] = prevState->pipeStageState[DECODE];
      prevState->pipeStageState[DECODE] = prevState->pipeStageState[FETCH];
      prevState->pc = nextRegisters->pc;
      pcState[DECODE] = pcState[FETCH];
      pcState[FETCH] = prevState->pc;
      SIM_MemInstRead(nextRegisters->pc, &newCmd);
      newState->cmd = newCmd;
      newState->src1Val = 0;
      newState->src2Val = 0;
      prevState->pipeStageState[FETCH] = *newState;
      fU[1] = fU[0];
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
  wbStage(nextData);
  memoryStage(isStallMem, isBranch, nextData);
  executeStage(nextData);
  fU[0].src1_valid = false;
  fU[0].src1_value = 0;
  fU[0].src2_valid = false;
  fU[0].src2_value = 0;
  decodeStage(isStallDec, nextData);
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
  int curMemDst = nextData.mem_index;
  int curExeDst = nextData.exe_index;
  bool isDstValid =false;

  //searching for src1Val
  if(pCurCmd->src1 != 0)
    curStage->src1Val = prevState->regFile[pCurCmd->src1];
  forwardUnit(&prevState->pipeStageState[MEMORY].cmd ,pCurCmd->src1 ,fU[0].src1_value,fU[0].src1_valid,isStall ,curMemDst , curExeDst);
  //searching for src2
  if (pCurCmd->isSrc2Imm){
    //  ~first -src2 is immediate
    curStage->src2Val = pCurCmd->src2;
  }
  //  ~second - check if memory load it now
  else{
    if(pCurCmd->src2 != 0)
      curStage->src2Val = prevState->regFile[pCurCmd->src2];
    forwardUnit(&prevState->pipeStageState[MEMORY].cmd ,pCurCmd->src2 ,fU[0].src2_value ,fU[0].src2_valid,isStall ,curMemDst , curExeDst);

  }
  //searching for dst
  if ((pCurCmd->opcode == CMD_STORE) || (pCurCmd->opcode == CMD_BR) || (pCurCmd->opcode == CMD_BREQ) || (pCurCmd->opcode == CMD_BRNEQ)){
    forwardUnit(&prevState->pipeStageState[MEMORY].cmd ,pCurCmd->dst ,nextData.dst_value,isDstValid ,isStall ,curMemDst , curExeDst);
  }
  return;
}

void forwardUnit (SIM_cmd* pCurCmd ,int &dstIndex ,int32_t& dstValue ,bool& dstValid , bool &isStall , int curMemDst , int curExeDst ){
  if(dstIndex == 0){
    dstValue = prevState->regFile[0];
    return;
  }
  SIM_cmd* exeCmd =&(prevState->pipeStageState[EXECUTE].cmd);
  //  ~first check if execute calculate it now
  if (curExeDst == dstIndex){
    if (forwarding){
      //forwarding is needed but couldn't be done here
      memForward = true;
    }
    else{
      // forwarding is not enabled - stall is needed
      isStall = true;
    }
  }
  else if ((exeCmd->dst == dstIndex) && ((exeCmd->opcode != CMD_BR) && (exeCmd->opcode != CMD_BRNEQ) && (exeCmd->opcode != CMD_BREQ) && (exeCmd->opcode != CMD_NOP))) {
    isStall = true;
  }
  //  ~seconnd check if memory load it now
  else if (curMemDst == dstIndex){
    if (pCurCmd->opcode == CMD_LOAD){ //if load it now
      if(forwarding){
        //forwarding is needed but couldn't be done here
        wbForward = true;
      }
      else {
        isStall = true;
      }
    }
    else{
      //transfered from the execute
      //  ~third forwardnig from the mem stage - calculated in the execute
      if(forwarding){
        dstValue = dataReg->exe_value;
        dstValid = true;
      }
      else{
        isStall = true;
      }
    }
  }
  //  ~ fourth - forwarding from the WB stage
  else if(dataReg->mem_index == dstIndex){
    if(forwarding || split_regfile){
      dstValue = dataReg->mem_value;
      dstValid = true;
    }
    else{
      isStall = true;
    }
  }
  else{
    dstValue = prevState->regFile[dstIndex];
  }
  return;
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

  if(fU[1].src1_valid){
    src1Val = fU[1].src1_value ;
    prevState->pipeStageState[EXECUTE].src1Val = src1Val;
  }
  if(fU[1].src2_valid){
    src2Val = fU[1].src2_value ;
    prevState->pipeStageState[EXECUTE].src2Val = src2Val;
  }

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
	SIM_cmd_opcode cmd = prevState->pipeStageState[MEMORY].cmd.opcode;
		int32_t src1Val = prevState->pipeStageState[MEMORY].src1Val;
		int32_t src2Val = prevState->pipeStageState[MEMORY].src2Val;
		nextData.mem_index = dataReg->exe_index;
		nextData.mem_value = dataReg->exe_value;

		//forward prev clk-tick exe result to current exe
		if (memForward) {
			if (cmd == CMD_ADD || cmd == CMD_SUB || cmd == CMD_ADDI || cmd == CMD_SUBI) { //it's an assert, but I don't want to add real asserts
				int mem_dst_reg = prevState->pipeStageState[MEMORY].cmd.dst;
				SIM_cmd exe_cmd = prevState->pipeStageState[EXECUTE].cmd;

				if (mem_dst_reg == exe_cmd.src1) {
					prevState->pipeStageState[EXECUTE].src1Val = dataReg->exe_value;
				}
        if (mem_dst_reg == exe_cmd.src2) {
					prevState->pipeStageState[EXECUTE].src2Val = dataReg->exe_value;
				}
        if (mem_dst_reg == exe_cmd.dst) {
					dataReg->dst_value = dataReg->exe_value;
				}
			}

		}

		switch(cmd){
		case CMD_BR:
			isBranch = true;
			break;
		case CMD_BREQ:
			if (src1Val == src2Val) {
				isBranch = true;
			}
			break;
		case CMD_BRNEQ:
			if (src1Val != src2Val) {
				isBranch = true;
			}
			break;
		case CMD_LOAD:
			if (SIM_MemDataRead(dataReg->exe_value, &nextData.mem_value) != 0) { //the read isn't complete
				nextData.mem_value = 0;
				isStall = true;
				return;
			} else {
				nextData.mem_index = prevState->pipeStageState[MEMORY].cmd.dst;
			}
			break;
		case CMD_STORE:
			SIM_MemDataWrite(dataReg->exe_value, src1Val);
			break;
		}

		return;

	//	//Check if branch is needed
	//	if (cmd == CMD_BR) {
	//		isBranch = true;
	//	}
	//	if (cmd == CMD_BREQ) {
	//		if (src1Val == src2Val) {
	//			isBranch = true;
	//		}
	//	} else if (cmd == CMD_BRNEQ) {
	//		if (src1Val != src2Val) {
	//			isBranch = true;
	//		}
	//		return;
	//	}
	//
	//	if (cmd == CMD_LOAD) {
	//		if (SIM_MemDataRead(dataReg->exe_value, &nextData.mem_value) != 0) { //the read isn't complete
	//			nextData.mem_value = 0;
	//			isStall = true;
	//			return;
	//		} else {
	//			nextData.mem_index = prevState->pipeStageState[MEMORY].cmd.dst;
	//		}
	//	}
	//
	//	if (cmd == CMD_STORE) {
	//		SIM_MemDataWrite(dataReg->exe_value, src1Val);
	//	}
	//
	//	return;
}

/* the Write Back Stage:
 ~ writing the data back to the registers (ADD.ADDI.SUB,SUBI,LOAD)
 */
void wbStage(dataStruct& nextData) {
	SIM_cmd_opcode cmd = prevState->pipeStageState[WRITEBACK].cmd.opcode;

	int wb_dst_reg;

	if (cmd == CMD_ADD || cmd == CMD_SUB || cmd == CMD_ADDI || cmd == CMD_SUBI
			|| cmd == CMD_LOAD) {
		wb_dst_reg = prevState->pipeStageState[WRITEBACK].cmd.dst;
		if (!split_regfile && !forwarding) {
			nextRegisters->regFile[wb_dst_reg] = dataReg->mem_value;
		} else {
			prevState->regFile[wb_dst_reg] = dataReg->mem_value;
		}
	}
	if (cmd == CMD_LOAD && wbForward) {
		SIM_cmd exe_cmd = prevState->pipeStageState[EXECUTE].cmd;
		if (wb_dst_reg == exe_cmd.src1) {
			prevState->pipeStageState[EXECUTE].src1Val = dataReg->mem_value;
		}
    if (wb_dst_reg == exe_cmd.src2) {
			prevState->pipeStageState[EXECUTE].src2Val = dataReg->mem_value;
		}
    if (wb_dst_reg == exe_cmd.dst) {
			dataReg->dst_value = dataReg->mem_value;
		}
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

