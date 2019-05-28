/***********************************************************************************************/
/*********************************  MIPS 5-stage pipeline implementation ***********************/
/***********************************************************************************************/
`include "constants.h"

module cpu(input clock, input reset);
 reg [31:0] PC; 
 reg [31:0] IFID_PCplus4;
 reg [31:0] IFID_instr;
 reg [31:0] IDEX_rdA, IDEX_rdB, IDEX_signExtend;
 reg [4:0]  IDEX_instr_rt, IDEX_instr_rs, IDEX_instr_rd;                            
 reg        IDEX_RegDst, IDEX_ALUSrc;
 reg [1:0]  IDEX_ALUcntrl;
 reg        IDEX_Branch, IDEX_MemRead, IDEX_MemWrite; 
 reg        IDEX_MemToReg, IDEX_RegWrite;                
 reg [4:0]  EXMEM_RegWriteAddr, EXMEM_instr_rd; 
 reg [31:0] EXMEM_ALUOut;
 reg        EXMEM_Zero;
 reg [31:0] EXMEM_MemWriteData;
 reg        EXMEM_Branch, EXMEM_MemRead, EXMEM_MemWrite, EXMEM_RegWrite, EXMEM_MemToReg;
 reg [31:0] MEMWB_DMemOut;
 reg [4:0]  MEMWB_RegWriteAddr, MEMWB_instr_rd; 
 reg [31:0] MEMWB_ALUOut;
 reg        MEMWB_MemToReg, MEMWB_RegWrite;               
 wire [31:0] instr, ALUInA, ALUInB, ALUOut, rdA, rdB, signExtend, DMemOut, wRegData, PCIncr;
 wire Zero, RegDst, MemRead, MemWrite, MemToReg, ALUSrc, RegWrite, Branch;
 wire [5:0] opcode, func;
 wire [4:0] instr_rs, instr_rt, instr_rd, RegWriteAddr;
 wire [3:0] ALUOp;
 wire [1:0] ALUcntrl;
 wire [15:0] imm;
 //lab6
 reg [4:0] IDEX_instr_shamt;
 wire [4:0] instr_shamt, ALUShamt;
 wire [1:0] ForwardA, ForwardB;
 wire [31:0] ALUInATemp, ALUInBTemp;
 wire RegDstTemp, BranchTemp, MemReadTemp, MemWriteTemp, MemToRegTemp, ALUSrcTemp, RegWriteTemp;
 wire PCWrite, IFIDWrite, hazardUnitMux;
 //lab7
 reg EXMEM_beq_bne;
 reg [31:0] IDEX_PCPlus4, EXMEM_Branch_ALUOut;
 reg IDEX_BranchSel;
 wire BranchSel, BranchSelTemp, notZero;
 wire Jump, PCSrc, bubble_idex;
 wire IFFlush, IDFlush;
 wire [31:0]  PCNew, PCNewTemp, PCPlus4, JumpPC;
 wire [31:0] instrTemp;
 wire IDEX_BranchTemp, IDEX_MemReadTemp, IDEX_MemWriteTemp, IDEX_RegWriteTemp, IDEX_MemToRegTemp;
 wire beq_bne;
 
/***************** Instruction Fetch Unit (IF)  ****************/
 always @(posedge clock or negedge reset)
  begin 
    if (reset == 1'b0)     
       PC <= -1;     
    else if (PC == -1)
       PC <= 0;
    else 
       if(PCWrite == 1'b1)	
          PC <= PCNew;
  end
  
  assign PCPlus4 = PC + 32'd4;
  
  assign PCNew = (PCSrc == 1'b0) ? PCNewTemp : EXMEM_Branch_ALUOut; //MUX for branch
  assign PCNewTemp = (Jump == 1'b0) ? PCPlus4 : JumpPC; // MUX for jump
  
  //Flush for IF
  or or1(IFFlush, Jump, PCSrc);
  assign instr = (IFFlush == 1'b0) ? instrTemp : `NOP;

  
  // IFID pipeline register
 always @(posedge clock or negedge reset)
  begin 
    if (reset == 1'b0)     
      begin
       IFID_PCplus4 <= 32'b0;    
       IFID_instr <= 32'b0;
    end 
    else 
      if(IFIDWrite == 1'b1)
      begin
           IFID_PCplus4 <= PCPlus4;
           IFID_instr <= instr;
      end
  end
  
// Instruction memory 1KB
Memory cpu_IMem(clock, reset, 1'b1, 1'b0, PC>>2, 32'b0, instrTemp);
  
  
  
  
  
/***************** Instruction Decode Unit (ID)  ****************/
assign opcode = IFID_instr[31:26];
assign func = IFID_instr[5:0];
assign instr_rs = IFID_instr[25:21];
assign instr_rt = IFID_instr[20:16];
assign instr_rd = IFID_instr[15:11];
//assign instr_shmat = IFID_instr[10:6];  does not work? 
assign imm = IFID_instr[15:0];
assign signExtend = {{16{imm[15]}}, imm};
assign JumpPC = {IFID_PCplus4[31:28], {IFID_instr[25:0], 2'b0}};

// Register file
RegFile cpu_regs(clock, reset, instr_rs, instr_rt, MEMWB_RegWriteAddr, MEMWB_RegWrite, wRegData, rdA, rdB);

  // IDEX pipeline register
 always @(posedge clock or negedge reset)
  begin 
    if (reset == 1'b0)
      begin
       IDEX_rdA <= 32'b0;    
       IDEX_rdB <= 32'b0;
       IDEX_signExtend <= 32'b0;
       IDEX_instr_rd <= 5'b0;
       IDEX_instr_rs <= 5'b0;
       IDEX_instr_rt <= 5'b0;
       IDEX_instr_shamt <= 5'b0; //the assignment produces undefined results
       IDEX_RegDst <= 1'b0;
       IDEX_ALUcntrl <= 2'b0;
       IDEX_ALUSrc <= 1'b0;
       IDEX_Branch <= 1'b0;
       IDEX_MemRead <= 1'b0;
       IDEX_MemWrite <= 1'b0;
       IDEX_MemToReg <= 1'b0;                  
       IDEX_RegWrite <= 1'b0;
	   IDEX_PCPlus4 <= 32'b0;
	   IDEX_BranchSel <= 1'b0;
      end 
    else 
      begin
       IDEX_rdA <= rdA;
       IDEX_rdB <= rdB;
       IDEX_signExtend <= signExtend;
       IDEX_instr_rd <= instr_rd;
       IDEX_instr_rs <= instr_rs;
       IDEX_instr_rt <= instr_rt;
       IDEX_instr_shamt <= IFID_instr[10:6]; //the assignment produces undefined results
       IDEX_RegDst <= RegDst;
       IDEX_ALUcntrl <= ALUcntrl;
       IDEX_ALUSrc <= ALUSrc;
       IDEX_Branch <= Branch;
       IDEX_MemRead <= MemRead;
       IDEX_MemWrite <= MemWrite;
       IDEX_MemToReg <= MemToReg;                  
       IDEX_RegWrite <= RegWrite;
	   IDEX_PCPlus4 <= IFID_PCplus4;
	   IDEX_BranchSel <= BranchSel;
      end
  end

// Main Control Unit 
control_main control_main (RegDstTemp,
                  BranchTemp,
				  BranchSelTemp,
                  MemReadTemp,
                  MemWriteTemp,
                  MemToRegTemp,
                  ALUSrcTemp,
                  RegWriteTemp,
				  Jump,
                  ALUcntrl,
                  opcode);
                  
// Instantiation of Control Unit that generates stalls goes here


 control_hazard_detection hazard_unit(.PCWrite(PCWrite),
				.IFIDWrite(IFIDWrite),
				.DetectionSel(hazardUnitMux),
				.IFIDRegister_Rs(instr_rs),
				.IFIDRegister_Rt(instr_rt),
				.IDEXRegister_Rt(IDEX_instr_rt),
				.IDEXMemRead(IDEX_MemRead));
				
or or2(IDFlush, hazardUnitMux, PCSrc);

 assign RegDst = (IDFlush == 1'b0) ? RegDstTemp : 1'b0;
 assign Branch = (IDFlush == 1'b0) ? BranchTemp : 1'b0;
 assign MemRead = (IDFlush == 1'b0) ? MemReadTemp : 1'b0;
 assign MemWrite = (IDFlush == 1'b0) ? MemWriteTemp : 1'b0;
 assign MemToReg = (IDFlush == 1'b0) ? MemToRegTemp : 1'b0;
 assign ALUSrc = (IDFlush == 1'b0) ? ALUSrcTemp : 1'b0;
 assign RegWrite = (IDFlush == 1'b0) ? RegWriteTemp : 1'b0;
 assign BranchSel = (IDFlush == 1'b0) ? BranchSelTemp : 1'b0;                          
/***************** Execution Unit (EX)  ****************/
                 
assign ALUInA = ALUInATemp; //ALUInATemp = IDEX_rdA
                 
assign ALUInB = (IDEX_ALUSrc == 1'b0) ? ALUInBTemp : IDEX_signExtend; //ALUInBTemp = IDEX_rdB

assign ALUShamt = IDEX_instr_shamt;

//  ALU
ALU  #32 cpu_alu(ALUOut, Zero, ALUInA, ALUInB, ALUShamt, ALUOp);

not notzero(notZero, Zero);
assign beq_bne = (IDEX_BranchSel == 1'b0) ? notZero : Zero;

assign RegWriteAddr = (IDEX_RegDst==1'b0) ? IDEX_instr_rt : IDEX_instr_rd;

 // EXMEM pipeline register
 always @(posedge clock or negedge reset)
  begin 
    if (reset == 1'b0)     
      begin
       EXMEM_ALUOut <= 32'b0;    
       EXMEM_RegWriteAddr <= 5'b0;
       EXMEM_MemWriteData <= 32'b0;
       EXMEM_beq_bne <= 1'b0;
       EXMEM_Branch <= 1'b0;
       EXMEM_MemRead <= 1'b0;
       EXMEM_MemWrite <= 1'b0;
       EXMEM_MemToReg <= 1'b0;                  
       EXMEM_RegWrite <= 1'b0;
      end 
    else 
      begin
       EXMEM_ALUOut <= ALUOut;    
       EXMEM_RegWriteAddr <= RegWriteAddr;
       EXMEM_MemWriteData <= ALUInBTemp; // ALUInBTemp=IDEX_rdB or fowarded data
       EXMEM_beq_bne <= beq_bne;
       EXMEM_Branch <= IDEX_BranchTemp;
       EXMEM_MemRead <= IDEX_MemReadTemp;
       EXMEM_MemWrite <= IDEX_MemWriteTemp;
       EXMEM_MemToReg <= IDEX_MemToRegTemp;                  
       EXMEM_RegWrite <= IDEX_RegWriteTemp;
	   EXMEM_Branch_ALUOut <= IDEX_PCPlus4 + (IDEX_signExtend<<2);
      end
  end
  
  assign bubble_idex = PCSrc;
  assign IDEX_BranchTemp = (bubble_idex == 1'b0) ? IDEX_Branch : 1'b0;
  assign IDEX_MemReadTemp = (bubble_idex == 1'b0) ? IDEX_MemRead : 1'b0;
  assign IDEX_MemWriteTemp = (bubble_idex == 1'b0) ? IDEX_MemWrite : 1'b0;
  assign IDEX_MemToRegTemp = (bubble_idex == 1'b0) ? IDEX_MemToReg : 1'b0;
  assign IDEX_RegWriteTemp = (bubble_idex == 1'b0) ? IDEX_RegWrite : 1'b0;
  
  // ALU control
  control_alu control_alu(ALUOp, IDEX_ALUcntrl, IDEX_signExtend[5:0]);
  
   // Instantiation of control logic for Forwarding goes here
  control_bypass_ex forwarding(.bypassA(ForwardA), .bypassB(ForwardB), // output for mux selection  :2 bits
			       .idex_rs(IDEX_instr_rs),
                               .idex_rt(IDEX_instr_rt), 	       // registers currently in EX part
			       .exmem_rd(EXMEM_RegWriteAddr),
                               .exmem_regwrite(EXMEM_RegWrite),        // regwrite signal and regwrite address of instruction in MEM part
			       .memwb_rd(MEMWB_RegWriteAddr),
                               .memwb_regwrite(MEMWB_RegWrite));       // regwrite signal and regwrite address of instruction in WB part

  //MUX for ALUInA											
  assign ALUInATemp = (ForwardA == 2'b00)? IDEX_rdA : ((ForwardA == 2'b01)? wRegData : EXMEM_ALUOut);						
  
  //MUX for ALUInB
  assign ALUInBTemp = (ForwardB == 2'b00)? IDEX_rdB : ((ForwardB == 2'b01)? wRegData : EXMEM_ALUOut); 

  and branchand (PCSrc, EXMEM_beq_bne, EXMEM_Branch);
/***************** Memory Unit (MEM)  ****************/  

// Data memory 1KB
Memory cpu_DMem(clock, reset, EXMEM_MemRead, EXMEM_MemWrite, EXMEM_ALUOut, EXMEM_MemWriteData, DMemOut);

// MEMWB pipeline register
 always @(posedge clock or negedge reset)
  begin 
    if (reset == 1'b0)     
      begin
       MEMWB_DMemOut <= 32'b0;    
       MEMWB_ALUOut <= 32'b0;
       MEMWB_RegWriteAddr <= 5'b0;
       MEMWB_MemToReg <= 1'b0;                  
       MEMWB_RegWrite <= 1'b0;
      end 
    else 
      begin
       MEMWB_DMemOut <= DMemOut;
       MEMWB_ALUOut <= EXMEM_ALUOut;
       MEMWB_RegWriteAddr <= EXMEM_RegWriteAddr;
       MEMWB_MemToReg <= EXMEM_MemToReg;                  
       MEMWB_RegWrite <= EXMEM_RegWrite;
      end
  end

  
  
  

/***************** WriteBack Unit (WB)  ****************/  
assign wRegData = (MEMWB_MemToReg == 1'b0) ? MEMWB_ALUOut : MEMWB_DMemOut;


endmodule
