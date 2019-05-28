`include "constants.h"

/************** Main control in ID pipe stage  *************/
module control_main(output reg RegDst,
                output reg Branch,
				output reg BranchSel,
                output reg MemRead,
                output reg MemWrite,  
                output reg MemToReg,  
                output reg ALUSrc,  
                output reg RegWrite, 
				output reg Jump,
                output reg [1:0] ALUcntrl,  
                input [5:0] opcode);

  always @(*) 
   begin
     case (opcode)
      `R_FORMAT: 
          begin 
            RegDst = 1'b1;
            MemRead = 1'b0;
            MemWrite = 1'b0;
            MemToReg = 1'b0;
            ALUSrc = 1'b0;
            RegWrite = 1'b1;
            Branch = 1'b0;   
            Jump = 1'b0;
	        BranchSel = 1'b0;			
            ALUcntrl  = 2'b10; // R             
          end
       `LW :   
           begin 
            RegDst = 1'b0;
            MemRead = 1'b1;
            MemWrite = 1'b0;
            MemToReg = 1'b1;
            ALUSrc = 1'b1;
            RegWrite = 1'b1;
            Branch = 1'b0;
		    Jump = 1'b0;
	        BranchSel = 1'b0;
            ALUcntrl  = 2'b00; // add
           end
        `SW :   
           begin 
            RegDst = 1'b0;
            MemRead = 1'b0;
            MemWrite = 1'b1;
            MemToReg = 1'b0;
            ALUSrc = 1'b1;
            RegWrite = 1'b0;
            Branch = 1'b0;
			Jump = 1'b0;
	        BranchSel = 1'b0;
            ALUcntrl  = 2'b00; // add
           end
       `BEQ:  
           begin 
            RegDst = 1'b0;
            MemRead = 1'b0;
            MemWrite = 1'b0;
            MemToReg = 1'b0;
            ALUSrc = 1'b0;
            RegWrite = 1'b0;
            Branch = 1'b1;
			Jump = 1'b0;
	        BranchSel = 1'b1;
            ALUcntrl = 2'b01; // sub
           end
       `ADDI:  
           begin 
            RegDst = 1'b0;
            MemRead = 1'b0;
            MemWrite = 1'b0;
            MemToReg = 1'b0;
            ALUSrc = 1'b1;
            RegWrite = 1'b1;
            Branch = 1'b0;
			Jump = 1'b0;
	        BranchSel = 1'b0;
            ALUcntrl = 2'b00; // add
           end 
		`BNE:
		   begin
		    RegDst = 1'b0;
            MemRead = 1'b0;
            MemWrite = 1'b0;
            MemToReg = 1'b0;
            ALUSrc = 1'b0;
            RegWrite = 1'b0;
            Branch = 1'b1;
	        Jump = 1'b0;
			BranchSel=1'b0;	//zero or not_zero for beq and bne
			ALUcntrl = 2'b01; // sub
			end
		`JUMP:
		   begin
		   RegWrite = 1'b0;
           RegDst = 1'b0;
		   ALUSrc = 1'b0;
	       MemWrite = 1'b0;
	       MemRead = 1'b0;
	       MemToReg = 1'b0;
	       Branch = 1'b0;
	       Jump = 1'b1;   
	       BranchSel = 1'b0;
	       ALUcntrl = 2'b00;
		   end
       default:
           begin
            RegDst = 1'b0;
            MemRead = 1'b0;
            MemWrite = 1'b0;
            MemToReg = 1'b0;
            ALUSrc = 1'b0;
            RegWrite = 1'b0;
			Branch = 1'b0;
			Jump = 1'b0;
			BranchSel = 1'b0;
            ALUcntrl = 2'b00; 
			end
      endcase
    end // always
endmodule


/**************** Module for Bypass Detection in EX pipe stage goes here  *********/
 module  control_bypass_ex(output reg [1:0] bypassA,
                       output reg [1:0] bypassB,
                       input [4:0] idex_rs,
                       input [4:0] idex_rt,
                       input [4:0] exmem_rd,
                       input [4:0] memwb_rd,
                       input       exmem_regwrite,
                       input       memwb_regwrite);
       
  /* Fill in module details */
  always @(*)
   begin
    if (memwb_regwrite == 1'b1 && memwb_rd != 4'b0 && memwb_rd == idex_rs && (exmem_rd != idex_rs || exmem_regwrite == 1'b0))
     bypassA = 2'b01;
    else if (exmem_regwrite == 1'b1 && exmem_rd != 4'b0000 && exmem_rd == idex_rs)
     bypassA = 2'b10;
    else
     bypassA = 2'b00;
   end

  always @(*)
   begin
    if (memwb_regwrite == 1'b1 && memwb_rd != 4'b0 && memwb_rd == idex_rt && (exmem_rd != idex_rt || exmem_regwrite == 1'b0))
     bypassB = 2'b01;
    else if (exmem_regwrite == 1'b1 && exmem_rd != 4'b0000 && exmem_rd == idex_rt)
     bypassB = 2'b10;	
    else
     bypassB = 2'b00;
   end

endmodule          
                       

/**************** Module for Stall Detection in ID pipe stage goes here  *********/
module control_hazard_detection(output reg PCWrite,
				output reg IFIDWrite, 
				output reg DetectionSel, 
				input [4:0] IFIDRegister_Rs, 
				input [4:0] IFIDRegister_Rt, 
				input [4:0] IDEXRegister_Rt,
				input IDEXMemRead);

  always @(*)
   if (IDEXMemRead == 1'b1 && (IDEXRegister_Rt == IFIDRegister_Rs || IDEXRegister_Rt == IFIDRegister_Rt))
    begin
     PCWrite = 1'b0;
     IFIDWrite = 1'b0;
     DetectionSel = 1'b1;
    end
  
  always @(*)
   if(IDEXMemRead == 1'b0 ||(IDEXRegister_Rt != IFIDRegister_Rs && IDEXRegister_Rt != IFIDRegister_Rt))
    begin
     PCWrite = 1'b1;
     IFIDWrite = 1'b1;
     DetectionSel = 1'b0;
    end

endmodule        
                       
/************** control for ALU control in EX pipe stage  *************/
module control_alu(output reg [3:0] ALUOp,                  
               input [1:0] ALUcntrl,
               input [5:0] func);

  always @(ALUcntrl or func)  
    begin
      case (ALUcntrl)
        2'b10: 
           begin
             case (func)
              `ADD: ALUOp  = 4'b0010; // add
              `SUB: ALUOp = 4'b0110; // sub
              `AND: ALUOp = 4'b0000; // and
              `OR: ALUOp = 4'b0001; // or
              `NOR: ALUOp = 4'b1100; // nor
              `SLT: ALUOp = 4'b0111; // slt
              `SLL: ALUOp = 4'b1000; // sll
              `SLLV: ALUOp = 4'b0011; //sllv
			  `XOR: ALUOp = 4'b1011; //xor
              default: ALUOp = 4'b0000;       
             endcase 
          end   
        2'b00: 
              ALUOp  = 4'b0010; // add
        2'b01: 
              ALUOp = 4'b0110; // sub
        default:
              ALUOp = 4'b0000;
     endcase
    end
endmodule
