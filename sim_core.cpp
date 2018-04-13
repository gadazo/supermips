/* 046267 Computer Architecture - Spring 2017 - HW #1 */
/* This file should hold your implementation of the CPU pipeline core simulator */

#include "sim_api.h"
#define NROW 2
#define NCOL 2
#define NOVALID -1
#define EXE 0
#define MEM 1
void decodeStage(PipeStageState& nextState1, SIM_coreState& curState,bool &isStall);
void executeStage(PipeStageState& nextState2);
void memoryStage(PipeStageState& nextState3, bool &isStall , bool &isBranch);
void wbStage(SIM_cmd *pCurCmd , forwardUnit& forUn , int* pDstVal);
void initFU(forwardUnit* pForUn);
bool checkForwarding()
SIM_coreState* curState; //maybe static
int32_t* pcState[SIM_PIPELINE_DEPTH];


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
		curState = new SIM_coreState;
		pcState = new int32_t[SIM_PIPELINE_DEPTH];
	} catch (std::bad_alloc) {
		return -1;
	}

	curState->pc = 0;

	for (int i=0 ; i<SIM_PIPELINE_DEPTH ; i++){
		curState->pipeStageState[i].cmd = CMD_NOP;
		curState->pipeStageState[i].src1Val = 0;
		curState->pipeStageState[i].src2Val = 0;
	}

	for (int i=0 ; i<SIM_REGFILE_SIZE ; i++){
		curState->regFile[i] = 0;
	}

	for (int i=0 ; i<SIM_PIPELINE_DEPTH ; i++){
		pcState[i] = 0;
	}
	return 0;
}

/*! SIM_CoreClkTick: Update the core simulator's state given one clock cycle.
  This function is expected to update the core pipeline given a clock cycle event.
*/
void SIM_CoreClkTick() {
	PipeStageState *nextState = new PipeStageState[SIM_PIPELINE_DEPTH];
	bool isStallDec = false;
	bool isStallMem = false;
	bool isBranch = false;
	SIM_cmd newCmd;
  int dstValDec[1][1] = {NOVALID,0}; //{isValid , value} the dst value for BRANCH, STORE commands
  int dstVal[NROW][NCOl] = {0}; //TODO: check if intialized
  /* that matrix is used to transfer values between the stages and for Forwarding

     Stage:                 EXE                            MEM
     dst+isValid:{        dst index                      dst index           }
     dst value : {     Arithmetic value       Arithmetic value / Load value  }

     if the value is not valid dst index will be <0
  */
	//fetch stage
	SIM_MemInstRead(curState->pc, &newCmd);
	nextState[0].cmd = newCmd;
	nextState[0].src1Val = 0;
	nextState[0].src2Val = 0;/
	//decode + RF
    decodeStage(nextState[1],curState, isStallDec , dstValDec , &dstVal);
	//Execute
	executeStage(nextState[2]);
	//Memory
	memoryStage(nextState[3], isStallMem , isBranch);
	//WB
	wbStage();
	//update PC
	if (isStallDec){
    //Stall - do not change the PC
	}
  else if (isStallMem){
    
  }
	else if (isBranch){
    //Branch - change the PC to the new dest
	}
  else{
      //PC+4
    }
}
typedef struct {
  bool valid1;
  bool valid2;
  bool validDst;
  int32_t val1;
  int32_t val2;
  inst32_t valDst;
} forwardUnit;

/* the Decode and RF stage:
  ~ read from the RF and update src(1/2)val

  Flags:
  ~ isStall = RAW - check if register is needed and is rewriten in previous cmds
*/
void decodeStage(PipeStageState& nextState1, SIM_coreState& curState, bool &isStall , int* pDstValDec , int* pDstVal){
  SIM_cmd *pCurCmd = curState.pipeStageState[FETCH].cmd; //TODO: check if right
  forwardUnit *pForUn = new forwardUnit;
  initFU(pforUn);
  //if stall is needed put the the curState[FETCH] to nextState[DECODE]
  bool checkForward = checkForwarding(pCurCmd ,pForUn ,pDstVal);
  if((!forwarding) && (checkForward)){
    isStall = true;
    nextState1 = curState.pipeStageState[DECODE];
    return;
  }
  //if -s or -f wbStage will executed before the decodeStage
    if(pForUn->valid1)
      nextState1.src1Val = pForUn->val1;
    else{
      nextState1.src1Val = curState.regFile[pCurCmd->src1];
    }
    if (!pCurCmd->isSrc2Imm){
      if(pForUn->valid2)
        nextState1.src2Val = pForUn->val2 ;
      else
        nextState1.src2Val = curState.regFile[pCurCmd->src2];
    }
    else {
      nextState1.src2Val = pCurCmd->src2;
    }
    if((pCurCmd->opcode == CMD_BR) || (pCurCmd->opcode == CMD_BREQ) || (pCurCmd->opcode == CMD_BRNEQ)){
      pDstValDec[0] = 1;
      if (pForUn->validDst)
        pDstValDec[1] = pForUn->valDst;
      else
        pDstValDec[1] = curState.regFile[pCurCmd->dst];
    }
}

bool checkForwarding(SIM_cmd *pCurCmd , forwardUnit& forUn , int* pDstVal){
  bool isForwardingNeeded = false;
  int reg1 = pCurCmd->src1;
  int reg2 = (pCurCmd->isSrc2Imm)? pCurCmd->src2 : NOVALID;
  if (pDstVal[EXE][0] == reg1){
    isForwardingNeeded = true;
    if(forwarding){
      forUn.valid1 = true;
      forUn.val1 = pDstVal[EXE][1];
    }
  }
  else if (pDstVal[MEM][0] == reg1){
    isForwardingNeeded = true;
    if(forwarding || split_regfile){
      forUn.valid1 = true;
      forUn.val1 = pDstVal[MEM][1];
    }
  }
  if (reg2 != NOVALID){
    if(pDstVal[EXE][0] == reg2){
      isForwardingNeeded = ture;
      if(forwarding){
        forUn.valid2 = true;
        forUn.val2 = pDstVal[EXE][1];
      }
    }
    else if (pDstVal[MEM][0] == reg2){
      isForwardingNeeded = true;
      if (forwarding || split_regfile){
        forUn.valid2 = true;
        forUn.val2 = pDstVal[MEM][1];
      }
    }
  }
  if((pCurCmd->opcode == CMD_BR) || (pCurCmd->opcode == CMD_BREQ) || (pCurCmd->opcode == CMD_BRNEQ)){
    int dst = pCurCmd->dst;
    if(pDstVal[EXE][0] == dst){
      isForwardingNeeded = ture;
      if(forwarding){
        forUn.validDst = true;
        forUn.valDst = pDstVal[EXE][1];
      }
    }
    else if (pDstVal[MEM][0] == dst){
      isForwardingNeeded = true;
      if (forwarding || split_regfile){
        forUn.validDst = true;
        forUn.valDst = pDstVal[MEM][1];
      }
    }
  }
  return isForwardingNeeded;
}


void initFU(forwardUnit* pForUn){
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
void executeStage(PipeStageState& nextState2){

}

/* the Memory Stage:
   ~ Check if branch is needed
   ~ Get information from the main memory

   Flags:
   ~ isStall = stall the pipe if couldn't retrieve data from the main memory
   ~ isBranch = true if branch is taken.
 */

void memoryStage(PipeStageState& nextState3, bool &isStall , bool &isBranch){

  //if stall is needed put the the curState[3] to nextState[3]

}

/* the Write Back Stage:
   ~ writing the data back to the registers (ADD.ADDI.SUB,SUBI,LOAD)
 */

void wbStage(){
  
}

/*! SIM_CoreGetState: Return the current core (pipeline) internal state
    curState: The returned current pipeline state
    The function will return the state of the pipe at the end of a cycle
*/
void SIM_CoreGetState(SIM_coreState *curState) {
}

