/* 046267 Computer Architecture - Spring 2017 - HW #1 */
/* This file should hold your implementation of the CPU pipeline core simulator */

#include "sim_api.h"

void decodeStage(PipeStageState& nextState1, bool &isStall);
void executeStage(PipeStageState& nextState2);
void memoryStage(PipeStageState& nextState3, bool &isStall , bool &isBranch);
void wbStage();

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
	//fetch stage

	SIM_MemInstRead(curState->pc, &newCmd);
	nextState[0].cmd = newCmd;
	nextState[0].src1Val = 0;
	nextState[0].src2Val = 0;
	//decode + RF
	decodeStage(nextState[1], isStallDec);
	//Execute
	executeStage(nextState[2]);
	//Memory
	memoryStage(nextState[3], isStallMem , isBranch);
	//WB
	wbStage();
	//update PC
	if ((isStallDec) || (isStallMem)){
	//Stall - do not change the PC
	}
	else if (isBranch){
    //Branch - change the PC to the new dest
	}
    else{
      //PC+4
    }
}

/* the Decode and RF stage:
  ~ read from the RF and update src(1/2)val

  Flags:
  ~ isStall = RAW - check if register is needed and is rewriten in previous cmds
*/
void decodeStage(PipeStageState& nextState1, bool &isStall){


  //if stall is needed put the the curState[1] to nextState[1]

  //if -s or -f wbStage will executed before the decodeStage
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

