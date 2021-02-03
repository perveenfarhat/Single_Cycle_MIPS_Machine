/*
 *
 * Redistributions of any form whatsoever must retain and/or include the
 * following acknowledgment, notices and disclaimer:
 *
 * This product includes software developed by Carnegie Mellon University. 
 *
 * Copyright (c) 2004 by Babak Falsafi and James Hoe,
 * Computer Architecture Lab at Carnegie Mellon (CALCM), 
 * Carnegie Mellon University.
 *
 * This source file was written and maintained by Jared Smolens 
 * as part of the Two-Way In-Order Superscalar project for Carnegie Mellon's 
 * Introduction to Computer Architecture course, 18-447. The source file
 * is in part derived from code originally written by Herman Schmit and 
 * Diana Marculescu.
 *
 * You may not use the name "Carnegie Mellon University" or derivations 
 * thereof to endorse or promote products derived from this software.
 *
 * If you modify the software you must place a notice on or within any 
 * modified version provided or made available to any third party stating 
 * that you have modified the software.  The notice shall include at least 
 * your name, address, phone number, email address and the date and purpose 
 * of the modification.
 *
 * THE SOFTWARE IS PROVIDED "AS-IS" WITHOUT ANY WARRANTY OF ANY KIND, EITHER 
 * EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO ANYWARRANTY 
 * THAT THE SOFTWARE WILL CONFORM TO SPECIFICATIONS OR BE ERROR-FREE AND ANY 
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, 
 * TITLE, OR NON-INFRINGEMENT.  IN NO EVENT SHALL CARNEGIE MELLON UNIVERSITY 
 * BE LIABLE FOR ANY DAMAGES, INCLUDING BUT NOT LIMITED TO DIRECT, INDIRECT, 
 * SPECIAL OR CONSEQUENTIAL DAMAGES, ARISING OUT OF, RESULTING FROM, OR IN 
 * ANY WAY CONNECTED WITH THIS SOFTWARE (WHETHER OR NOT BASED UPON WARRANTY, 
 * CONTRACT, TORT OR OTHERWISE).
 *
 */

// Include the MIPS constants
`include "mips_defines.vh"
`include "internal_defines.vh"

////
//// mips_decode: Decode MIPS instructions
////
//// op      (input)  - Instruction opcode
//// funct2  (input)  - Instruction minor opcode
//// rt      (input)  - Instruction minor opcode
//// alu_sel (output) - Selects the ALU function
//// we      (output) - Write to the register file
//// Sys     (output) - System call exception
//// RI      (output) - Reserved instruction exception
//// DesReg  (output) - Destination register rt(1) or rd(0)?
//// op1src  (output)- first opearnd source mux select control
//// op2src  (output)- second opearnd source mux select controls
//// isbranch		- true if branch inst
//// isjump		- true if jump inst
//// jal		- 1 when r[31] stores pc+4
//// jalr		- jump address in target or reg?
module mips_decode(/*AUTOARG*/
   // Outputs
   ctrl_we, ctrl_Sys, ctrl_RI, alu__sel, ctrl_DesReg, ctrl_op1src, ctrl_op2src, ctrl_isbranch, ctrl_isjump, ctrl_jal, ctrl_jalr, ctrl_load_imm, ctrl_mem2reg, ctrl_sw_memwren, ctrl_sb_memwren, ctrl_sh_memwren, ctrl_lw_load, ctrl_lb_load, ctrl_lh_load, ctrl_load_se, ctrl_mem_load_size,
   // Inputs
   dcd_op, dcd_rt, dcd_funct2
   );

   input       [5:0] dcd_op, dcd_funct2;
   input       [4:0] dcd_rt;
   output reg        ctrl_we, ctrl_Sys, ctrl_RI, ctrl_DesReg, ctrl_op1src, ctrl_isbranch, ctrl_isjump, ctrl_jal, ctrl_jalr, ctrl_load_imm, ctrl_mem2reg, ctrl_load_se, ctrl_sw_memwren, ctrl_sb_memwren, ctrl_sh_memwren, ctrl_lw_load, ctrl_lb_load, ctrl_lh_load;
   output reg  [3:0] alu__sel ;
   output reg  [1:0] ctrl_op2src, ctrl_mem_load_size;

   always @(*) begin
     alu__sel = 4'hx;
     ctrl_we = 1'b0;
     ctrl_Sys = 1'b0;
     ctrl_RI = 1'b0;
     ctrl_DesReg = 1'b0;
     ctrl_op1src = 1'b0;
     ctrl_op2src = 2'b00;
     ctrl_isbranch = 1'b0;
     ctrl_isjump = 1'b0;
     ctrl_jal = 1'b0;
     ctrl_jalr = 1'b0;
     ctrl_load_imm = 1'b0;
     ctrl_mem2reg = 1'b0;
     ctrl_sw_memwren = 1'b0;
     ctrl_sh_memwren = 1'b0;
     ctrl_sb_memwren = 1'b0;
     ctrl_lw_load = 1'b0;
     ctrl_lh_load = 1'b0;
     ctrl_lb_load = 1'b0;
     ctrl_mem_load_size = 2'b00;
     ctrl_load_se = 1'b0;
     case(dcd_op)
       `OP_OTHER0:
         case(dcd_funct2)
           `OP0_SYSCALL:
                ctrl_Sys = 1'b1;
	   `OP0_ADD:
	      begin
		alu__sel = `ALU_ADD;
		ctrl_we = 1'b1;
	      end
	   `OP0_ADDU:	
	      begin
		alu__sel = `ALU_ADD;
		ctrl_we = 1'b1;
	      end
	   `OP0_SUB:	
	      begin
		alu__sel = `ALU_SUB;
		ctrl_we = 1'b1;
	      end
	   `OP0_SUBU:	
	      begin
		alu__sel = `ALU_SUB;
		ctrl_we = 1'b1;
	      end
	   `OP0_AND:	
	      begin
		alu__sel = `ALU_AND;
		ctrl_we = 1'b1;
	      end
	   `OP0_OR:	
	      begin
		alu__sel = `ALU_OR;
		ctrl_we = 1'b1;
	      end
	   `OP0_NOR:	
	      begin
		alu__sel = `ALU_NOR;
		ctrl_we = 1'b1;
	      end
	   `OP0_XOR:	
	      begin
		alu__sel = `ALU_XOR;
		ctrl_we = 1'b1;
	      end
	   `OP0_SLT:	
	      begin
		alu__sel = `ALU_SLT;
		ctrl_we = 1'b1;
	      end
	   `OP0_SLTU:	
	      begin
		alu__sel = `ALU_SLT;
		ctrl_we = 1'b1;
	      end
	   `OP0_SLL:	
	      begin
		alu__sel = `ALU_SLL;
		ctrl_we = 1'b1;
		ctrl_op1src = 1'b1;
	      end
	   `OP0_SRL:	
	      begin
		alu__sel = `ALU_SRL;
		ctrl_we = 1'b1;
		ctrl_op1src = 1'b1;
	      end
	   `OP0_SRA:	
	      begin
		alu__sel = `ALU_SRL;
		ctrl_we = 1'b1;
		ctrl_op1src = 1'b1;
	      end
          `OP0_JALR:
            begin
               ctrl_we = 1'b1;
               ctrl_isjump = 1'b1;
               ctrl_jal = 1'b1;
               ctrl_jalr = 1'b1;
            end
          `OP0_JR:
            begin
               ctrl_we = 1'b0;
               ctrl_isjump = 1'b1;
               ctrl_jal = 1'b0;
               ctrl_jalr = 1'b1;
            end
           default:
                ctrl_RI = 1'b1;
         endcase // (dcd_funct2)
       `OP_OTHER1:
         case(dcd_rt)
           `OP1_BLTZ:
             begin
                ctrl_we = 1'b0;
                ctrl_isbranch = 1'b1;
                ctrl_jal = 1'b0;
                alu__sel = `ALU_BLTZ;
             end
           `OP1_BLTZAL:
             begin
                ctrl_we = 1'b1;
                ctrl_isbranch = 1'b1;
                ctrl_jal = 1'b1;
                alu__sel = `ALU_BLTZ;
             end
           `OP1_BGEZ:
             begin
                ctrl_we = 1'b0;
                ctrl_isbranch = 1'b1;
                alu__sel = `ALU_BGEZ;
             end
           `OP1_BGEZAL:
             begin
                ctrl_we = 1'b1;
                ctrl_isbranch = 1'b1;
                ctrl_jal = 1'b1;
                alu__sel = `ALU_BGEZ;
             end
           default:
                ctrl_RI = 1'b1;
        endcase// (dcd_rt)
       `OP_ADDIU:
         begin
            alu__sel = `ALU_ADD;
            ctrl_we = 1'b1;
	    ctrl_op2src = 2'b01;
     	    ctrl_DesReg = 1'b1;
         end
       `OP_ADDI:
         begin
            alu__sel = `ALU_ADD;
            ctrl_we = 1'b1;
	    ctrl_op2src = 2'b01;
     	    ctrl_DesReg = 1'b1;
         end
       `OP_ADDIU:
         begin
            alu__sel = `ALU_ADD;
            ctrl_we = 1'b1;
	    ctrl_op2src = 2'b01;
     	    ctrl_DesReg = 1'b1;
         end
       `OP_ANDI:
         begin
            alu__sel = `ALU_AND;
            ctrl_we = 1'b1;
	    ctrl_op2src = 2'b10;
     	    ctrl_DesReg = 1'b1;
         end
       `OP_ORI:
         begin
            alu__sel = `ALU_OR;
            ctrl_we = 1'b1;
	    ctrl_op2src = 2'b10;
     	    ctrl_DesReg = 1'b1;
         end
       `OP_XORI:
         begin
            alu__sel = `ALU_XOR;
            ctrl_we = 1'b1;
	    ctrl_op2src = 2'b10;
     	    ctrl_DesReg = 1'b1;
         end
       `OP_SLTI:
         begin
            alu__sel = `ALU_SLT;
            ctrl_we = 1'b1;
	    ctrl_op2src = 2'b01;
     	    ctrl_DesReg = 1'b1;
         end
       `OP_SLTIU:
         begin
            alu__sel = `ALU_SLT;
            ctrl_we = 1'b1;
	    ctrl_op2src = 2'b01;
     	    ctrl_DesReg = 1'b1;
         end
       `OP_BEQ:
         begin
            alu__sel = `ALU_BEQ;
            ctrl_we = 1'b0;
	    ctrl_isbranch = 1'b1;
         end
       `OP_BNE:
         begin
            alu__sel = `ALU_BNE;
            ctrl_we = 1'b0;
	    ctrl_isbranch = 1'b1;
         end
       `OP_J:
         begin
            ctrl_we = 1'b0;
	    ctrl_isjump = 1'b1;
         end
       `OP_JAL:
         begin
            ctrl_we = 1'b1;
	    ctrl_isjump = 1'b1;
	    ctrl_jal = 1'b1;
         end
       `OP_BGTZ:
         begin
            ctrl_we = 1'b0;
	    ctrl_isbranch = 1'b1;
	    ctrl_jal = 1'b0;
	    alu__sel = `ALU_BGTZ;
         end
       `OP_BLEZ:
         begin
            ctrl_we = 1'b0;
	    ctrl_isbranch = 1'b1;
	    ctrl_jal = 1'b0;
	    alu__sel = `ALU_BLEZ;
         end
       `OP_LUI:
         begin
            ctrl_we = 1'b1;
	    ctrl_load_imm = 1'b1;
	    ctrl_DesReg = 1'b1;
         end
       `OP_LW:
         begin
            ctrl_we = 1'b1;
	    alu__sel = `ALU_ADD;
	    ctrl_DesReg = 1'b1;
            ctrl_op2src = 1'b01;
	    ctrl_mem2reg = 1'b1;
	    ctrl_lw_load = 1'b1;
         end
       `OP_LB:
         begin
            ctrl_we = 1'b1;
	    alu__sel = `ALU_ADD;
	    ctrl_DesReg = 1'b1;
            ctrl_op2src = 2'b01;
	    ctrl_mem2reg = 1'b1;
	    ctrl_load_se = 1'b1;
	    ctrl_mem_load_size = 2'b01;
	    ctrl_lb_load = 1'b1;
         end
       `OP_LBU:
         begin
            ctrl_we = 1'b1;
	    alu__sel = `ALU_ADD;
	    ctrl_DesReg = 1'b1;
            ctrl_op2src = 2'b01;
	    ctrl_mem2reg = 1'b1;
	    ctrl_load_se = 1'b0;
	    ctrl_mem_load_size = 2'b01;
	    ctrl_lb_load = 1'b1;
         end
       `OP_LHU:
         begin
            ctrl_we = 1'b1;
	    alu__sel = `ALU_ADD;
	    ctrl_DesReg = 1'b1;
            ctrl_op2src = 2'b01;
	    ctrl_mem2reg = 1'b1;
	    ctrl_load_se = 1'b0;
	    ctrl_mem_load_size = 2'b10;
	    ctrl_lh_load = 1'b1;
         end
       `OP_LH:
         begin
            ctrl_we = 1'b1;
	    alu__sel = `ALU_ADD;
	    ctrl_DesReg = 1'b1;
            ctrl_op2src = 2'b01;
	    ctrl_mem2reg = 1'b1;
	    ctrl_load_se = 1'b1;
	    ctrl_mem_load_size = 2'b10;
	    ctrl_lh_load = 1'b1;
         end
       `OP_SW:
         begin
            ctrl_we = 1'b0;
	    alu__sel = `ALU_ADD;
	    ctrl_DesReg = 1'b1;
            ctrl_op2src = 2'b01;
	    ctrl_mem2reg = 1'b0;
	    ctrl_sw_memwren = 1'b1;
         end
       `OP_SB:
         begin
            ctrl_we = 1'b0;
	    alu__sel = `ALU_ADD;
	    ctrl_DesReg = 1'b1;
            ctrl_op2src = 2'b01;
	    ctrl_mem2reg = 1'b0;
	    ctrl_sb_memwren = 1'b1;
         end
       `OP_SH:
         begin
            ctrl_we = 1'b0;
	    alu__sel = `ALU_ADD;
	    ctrl_DesReg = 1'b1;
            ctrl_op2src = 2'b01;
	    ctrl_mem2reg = 1'b0;
	    ctrl_sh_memwren = 1'b1;
         end
       default:
         begin
            ctrl_RI = 1'b1;
         end
     endcase // case(op)
   end

endmodule
