/* 046267 Computer Architecture - Spring 2017 - HW #1 */
/* This file should hold your implementation of the CPU pipeline core simulator */

#include "sim_api.h"

//NETA!

/*! SIM_CoreReset: Reset the processor core simulator machine to start new simulation
  Use this API to initialize the processor core simulator's data structures.
  The simulator machine must complete this call with these requirements met:
  - PC = 0  (entry point for a program is at address 0)
  - All the register file is cleared (all registers hold 0)
  - The value of IF is the instuction in address 0x0
  \returns 0 on success. <0 in case of initialization failure.
*/
int SIM_CoreReset(void) {
}

/*! SIM_CoreClkTick: Update the core simulator's state given one clock cycle.
  This function is expected to update the core pipeline given a clock cycle event.
*/
void SIM_CoreClkTick() {
  pipeStageState *nextState = new pipeStageState[SIM_PIPELINE_DEPTH];
  bool isStallDec = false;
  bool isStallMem = false;
  bool isBranch = false;
  SIM_cmd newCmd;
  //fetch stage
  SIM_MemInstRead(curState->pc, newCmd);
  nextState[0].cmd = newCmd;
  nextState[0].src1Val = 0;
  nextState[0].src2Val = 0;
  //decode + RF
  decodeStage(isStallDec);
  //Execute
  executeStage();
  //Memory
  memoryStage(isStallMem , isBranch);
  //WB
  wbStage();
  //update PC
  if ((isStallDec) || (isStallMem)){
    //Stall - do not change the PC
  }
  else if (isBranch){
    //Branch - change the PC to the new dest
    else{
      //PC+4
    }
  }
}

/* the Decode and RF stage:
  ~ read from the RF and update src(1/2)val

  Flags:
  ~ isStall = RAW - check if register is needed and is rewriten in previous cmds
*/
void decodeStage (bool &isStall){


  //if stall is needed put the the curState[1] to nextState[1]

  //if -s or -f wbStage will executed before the decodeStage
}

/* The Execute stage:
   ~ Arithmetic operations (ADD,SUB,ADDI,SUBI)
   ~ Comparison operations (BREQ,BRNEQ)
   ~ Branch Destination Calculations (BR,BREQ,BRNEQ)
   ~ Memory Destination Calculations (LOAD,STORE)
*/
void executeStage(){
  
}

/* the Memory Stage:
   ~ Check if branch is needed
   ~ Get information from the main memory

   Flags:
   ~ isStall = stall the pipe if couldn't retrieve data from the main memory
   ~ isBranch = true if branch is taken.
 */

void memoryStage(bool &isStall , bool &isBranch){

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

