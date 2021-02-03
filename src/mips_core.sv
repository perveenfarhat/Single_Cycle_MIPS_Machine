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
 *xo
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

//////
////// MIPS 447: A single-cycle MIPS ISA simulator
//////

// Include the MIPS constants
`include "mips_defines.vh"
`include "internal_defines.vh"

////
//// The MIPS standalone processor module
////
////   clk          (input)  - The clock
////   inst_addr    (output) - Address of instruction to load
////   inst         (input)  - Instruction from memory
////   inst_excpt   (input)  - inst_addr not valid
////   mem_addr     (output) - Address of data to load
////   mem_data_in  (output) - Data for memory store
////   mem_data_out (input)  - Data from memory load
////   mem_write_en (output) - Memory write mask
////   mem_excpt    (input)  - mem_addr not valid
////   halted       (output) - Processor halted
////   reset        (input)  - Reset the processor
////   

module mips_core(/*AUTOARG*/
   // Outputs
   inst_addr, mem_addr, mem_data_in, mem_write_en, halted,
   // Inputs
   clk, inst_excpt, mem_excpt, inst, mem_data_out, rst_b
   );
   
   parameter text_start  = 32'h00400000; /* Initial value of $pc */

   // Core Interface
   input         clk, inst_excpt, mem_excpt;
   output [29:0] inst_addr;
   output [29:0] mem_addr;
   input  [31:0] inst, mem_data_out;
   output logic [31:0] mem_data_in;
   output logic [3:0]  mem_write_en;
   output        halted;
   input         rst_b;

   // Forced interface signals -- required for synthesis to work OK.
   // This is probably not what you want!
   //assign        mem_addr = 0;
   //assign        mem_data_in = mem_data_out;
   //assign        mem_write_en = 4'b0;

   // Internal signals
   wire [31:0]   pc, nextpc, nextnextpc, nextpcregin;
   wire          exception_halt, syscall_halt, internal_halt, alu__outbr, branchAndcond;
   wire          load_epc, load_bva, load_bva_sel;
   logic [31:0]   rt_data, rs_data, rd_data, alu__out, r_v0, addrTobranch, jumpAddress;
   wire [31:0]   epc, cause, bad_v_addr;
   wire [4:0]    cause_code;
   

   // Decode signals
   logic [31:0]   dcd_se_imm, dcd_se_offset, dcd_e_imm, dcd_se_mem_offset, alu_op1, alu_op2, dcd_jmp_target, dcd_load_imm, me_se_byte, mem_e_byte, mem_se_hw, mem_e_hw, mem_load_data, mem_signOrunsign;
   wire [5:0]    dcd_op, dcd_funct2;
   wire [4:0]    dcd_rs, desReg_num, dcd_funct1, dcd_rt, dcd_rd, dcd_shamt;
   wire [15:0]   dcd_offset, dcd_imm;
   wire [25:0]   dcd_target;
   wire [19:0]   dcd_code;
   wire          dcd_bczft;
   
   // PC Management
   register #(32, text_start) PCReg(pc, nextpcregin, clk, ~internal_halt, rst_b);
   register #(32, text_start+4) PCReg2(nextpc, nextnextpc, clk,
                                       ~internal_halt, rst_b);
   add_const #(4) NextPCAdder(nextnextpc, nextpcregin);
   assign        inst_addr = pc[31:2];

   // Instruction decoding
   assign        dcd_op = inst[31:26];    // Opcode
   assign        dcd_rs = inst[25:21];    // rs field
   assign        dcd_rt = inst[20:16];    // rt field
   assign        dcd_rd = inst[15:11];    // rd field
   assign        dcd_shamt = inst[10:6];  // Shift amount
   assign        dcd_bczft = inst[16];    // bczt or bczf?
   assign        dcd_funct1 = inst[4:0];  // Coprocessor 0 function field
   assign        dcd_funct2 = inst[5:0];  // funct field; secondary opcode
   assign        dcd_offset = inst[15:0]; // offset field
        // Sign-extended offset for branches
   assign        dcd_se_offset = { {14{dcd_offset[15]}}, dcd_offset, 2'b00 };
        // Sign-extended offset for load/store
   assign        dcd_se_mem_offset = { {16{dcd_offset[15]}}, dcd_offset };
   assign        dcd_imm = inst[15:0];        // immediate field
   assign        dcd_e_imm = { 16'h0, dcd_imm };  // zero-extended immediate
   assign        dcd_load_imm = { dcd_imm, 16'b0 };  // zero-extended immediate
        // Sign-extended immediate
   assign        dcd_se_imm = { {16{dcd_imm[15]}}, dcd_imm };
   assign        dcd_target = inst[25:0];     // target field
   assign        dcd_code = inst[25:6];       // Breakpoint code
	// decode address for jump
  assign	dcd_jmp_target = {pc[31:28] , dcd_target , 2'b00};

   // synthesis translate_off
   always @(posedge clk) begin
     // useful for debugging, you will want to comment this out for long programs
     if (rst_b) begin
       $display ( "=== Simulation Cycle %d ===", $time );
       $display ( "[pc=%x, inst=%x] [op=%x, rs=%d, rt=%d, rd=%d, imm=%x, f2=%x] [reset=%d, halted=%d]",
                   pc, inst, dcd_op, dcd_rs, dcd_rt, dcd_rd, dcd_imm, dcd_funct2, ~rst_b, halted);
     end
   end
   // synthesis translate_on

   // Let Verilog-Mode pipe wires through for us.  This is another example
   // of Verilog-Mode's power -- undeclared nets get AUTOWIREd up when we
   // run 'make auto'.
   
   /*AUTOWIRE*/
   // Beginning of automatic wires (for undeclared instantiated-module outputs)
   wire [3:0]		alu__sel;		// From Decoder of mips_decode.v
   wire			ctrl_RI;		// From Decoder of mips_decode.v
   wire			ctrl_Sys;		// From Decoder of mips_decode.v
   wire			ctrl_we;		// From Decoder of mips_decode.v
   wire		 	ctrl_DesReg;		// From Decoder of mips_decode.v
   wire		 	ctrl_op1src;		// From Decoder of mips_decode.v
   wire	[1:0]	 	ctrl_op2src;		// From Decoder of mips_decode.v
   wire			ctrl_isbranch;		// From Decoder of mips_decode.v
   wire			ctrl_isjump;		// From Decoder of mips_decode.v
   wire			ctrl_jal;		// From Decoder of mips_decode.v
   wire			ctrl_jalr;		// From Decoder of mips_decode.v
   wire			ctrl_load_imm;		// From Decoder of mips_decode.v
   wire			ctrl_sw_memwren;		// From Decoder of mips_decode.v
   wire			ctrl_sb_memwren;		// From Decoder of mips_decode.v
   wire			ctrl_sh_memwren;		// From Decoder of mips_decode.v
   wire			ctrl_load_se;    
   wire [1:0]		ctrl_mem_load_size;    
   // End of automatics

   // Generate control signals
   mips_decode Decoder(/*AUTOINST*/
		       // Outputs
		       .ctrl_we		(ctrl_we),
		       .ctrl_Sys	(ctrl_Sys),
		       .ctrl_RI		(ctrl_RI),
		       .alu__sel	(alu__sel[3:0]),
		       .ctrl_DesReg	(ctrl_DesReg),
		       .ctrl_op1src	(ctrl_op1src),
		       .ctrl_op2src	(ctrl_op2src),
		       .ctrl_isbranch	(ctrl_isbranch),
		       .ctrl_isjump	(ctrl_isjump),
		       .ctrl_jal	(ctrl_jal),
		       .ctrl_jalr	(ctrl_jalr),
		       .ctrl_load_imm	(ctrl_load_imm),
		       .ctrl_mem2reg	(ctrl_mem2reg),
		       .ctrl_sw_memwren    (ctrl_sw_memwren),
		       .ctrl_sb_memwren    (ctrl_sb_memwren),
		       .ctrl_sh_memwren    (ctrl_sh_memwren),
		       .ctrl_lw_load    (ctrl_lw_load),
		       .ctrl_lb_load    (ctrl_lb_load),
		       .ctrl_lh_load    (ctrl_lh_load),
		       .ctrl_load_se    (ctrl_load_se),
		       .ctrl_mem_load_size    (ctrl_mem_load_size),
		       // Inputs
		       .dcd_op		(dcd_op[5:0]),
		       .dcd_rt		(dcd_rt[4:0]),
		       .dcd_funct2	(dcd_funct2[5:0]));
 
   // Register File
   // Instantiate the register file from reg_file.v here.
   assign desReg_num[4:0] = ctrl_jal ? 5'd31 : (ctrl_DesReg ? dcd_rt[4:0] : dcd_rd[4:0]) ;
   assign rd_data = ctrl_load_imm ?  dcd_load_imm : (ctrl_jal ? nextpc : ctrl_mem2reg ? mem_load_data : alu__out);
   regfile reg1(.rs_data (rs_data[31:0]),
				.rt_data (rt_data[31:0]),
				.rs_num (dcd_rs[4:0]),
				.rt_num (dcd_rt[4:0]),
				.rd_num (desReg_num[4:0]),
				.rd_data (rd_data[31:0]),
				.rd_we (ctrl_we),
				.clk (clk),
				.rst_b (rst_b),
				.halted (halted));
   // Don't forget to hookup the "halted" signal to trigger the register dump 
 
   // synthesis translate_off
   initial begin
     // Delete this block when you are ready to try for real
     $display(""); 
     $display(""); 
     $display(""); 
     $display(""); 
     $display(">>>>> This works much better after you have hooked up the reg file. <<<<<");
     $display(""); 
     $display(""); 
     $display(""); 
     $display("");
	  #5000;
	  
     $finish;
   end
   // synthesis translate_on
   always_comb
     begin
	casez(1'b1)
	 ctrl_sw_memwren:
	      begin
		mem_write_en = 4'b1111; 
                mem_data_in = rt_data;
             end
	 ctrl_sb_memwren:
	      begin
		mem_write_en = 4'b0001 << alu__out[1:0]; 
                mem_data_in =  { 4{rt_data[7:0]} };
             end
	 ctrl_sh_memwren:
	      begin
		mem_write_en = 4'b0011 << alu__out[1:0];
                mem_data_in =  alu__out[0] ? { 8'b0,rt_data[15:0],8'b0 } : { 2{rt_data[15:0]} } ;
             end
	 default: 
	      begin
		mem_write_en = 4'b0000;
                mem_data_in = rt_data;
             end
       endcase 
   end

   /// memory instruction execute
   assign        mem_addr = alu__out[31:2];
   
   //assign        mem_write_en = ctrl_memwren;
   //assign	 mem_se_byte = { {24{mem_data_out[7]}} , mem_data_out[7:0] };
   //assign	 mem_e_byte = { {24{1'b0}} , mem_data_out[7:0] };
   //assign	 mem_se_hw = { {16{mem_data_out[15]}} , mem_data_out[15:0] };
   //assign	 mem_e_hw = { {16{1'b0}} , mem_data_out[15:0] };
 
  

   

   /// mux for first alu operand, choose rs_data ow rt_data
   ///
   assign alu_op1 = ctrl_op1src ? rt_data : rs_data ;

   /// mux for second alu operand, rt/imm
   ///
   mips_mux4x1 mux1(.out(alu_op2),
		    .sel(ctrl_op2src),
		    .in1(rt_data),
		    .in2(dcd_se_imm),
		    .in3(dcd_e_imm));

   // Execute
   mips_ALU ALU(.alu__out(alu__out), 
                .alu__op1(alu_op1),
                .alu__op2(alu_op2),
                .alu__sel(alu__sel),
		.alu__shiftby(dcd_shamt),
		.alu__outbr(alu__outbr));

  // load memory mux
    always_comb
      begin
	casez(1'b1)
	  ctrl_lw_load:
	     mem_load_data = mem_data_out;
	  ctrl_lb_load:
	   begin
	     case(alu__out[1:0])
		2'b00:
	     	  mem_load_data = ctrl_load_se ? { {24{mem_data_out[7]}} ,mem_data_out[7:0]} : { 24'b0 , mem_data_out[7:0] } ;
		2'b01:
	     	  mem_load_data = ctrl_load_se ? { {24{mem_data_out[15]}} ,mem_data_out[15:8]} : { 24'b0 , mem_data_out[15:8] } ;
		2'b10:
	     	  mem_load_data = ctrl_load_se ? { {24{mem_data_out[23]}} ,mem_data_out[23:16]} : { 24'b0 , mem_data_out[23:16] } ;
		2'b11:
	     	  mem_load_data = ctrl_load_se ? { {24{mem_data_out[31]}} ,mem_data_out[31:24]} : { 24'b0 , mem_data_out[31:24] } ;
	      endcase
	    end
	  ctrl_lh_load:
	   begin
	     case(alu__out[1:0])
		2'b00:
	     	  mem_load_data = ctrl_load_se ? { {16{mem_data_out[15]}} ,mem_data_out[15:0]} : { 15'b0 , mem_data_out[15:0] } ;
		2'b01:
	     	  mem_load_data = ctrl_load_se ? { {16{mem_data_out[23]}} ,mem_data_out[23:8]} : { 15'b0 , mem_data_out[23:8] } ;
		2'b10:
	     	  mem_load_data = ctrl_load_se ? { {16{mem_data_out[31]}} ,mem_data_out[31:16]} : { 15'b0 , mem_data_out[31:16] } ;
		default:
	     	  mem_load_data = ctrl_load_se ? { {16{mem_data_out[15]}} ,mem_data_out[15:0]} : { 15'b0 , mem_data_out[15:0] } ;
	      endcase
	    end
	endcase
     end
		   
	     //mem_load_data = mem_data_out;
//    mips_mux4x1 loadmux(.out(mem_load_data),
//		      .sel(ctrl_mem_load_size),
//		      .in1(mem_data_out),
//		      .in2(ctrl_load_se ? mem_se_byte : mem_e_byte),
//		      .in3(ctrl_load_se ? mem_se_hw : mem_e_hw));
//
   // Execute Branch
    assign jumpAddress = ctrl_jalr ? rs_data : dcd_jmp_target; 
    assign addrTobranch = dcd_se_offset + nextpc;
    assign branchAndcond = (ctrl_isbranch & alu__outbr);
    mips_mux4x1 pcmux(.out(nextpcregin),
		      .sel({branchAndcond,ctrl_isjump}),
		      .in1(nextpc),
		      .in2(jumpAddress),
		      .in3(addrTobranch));
 

   // Miscellaneous stuff (Exceptions, syscalls, and halt)
   exception_unit EU(.exception_halt(exception_halt), .pc(pc), .rst_b(rst_b),
                     .clk(clk), .load_ex_regs(load_ex_regs),
                     .load_bva(load_bva), .load_bva_sel(load_bva_sel),
                     .cause(cause_code),
                     .IBE(inst_excpt),
                     .DBE(1'b0),
                     .RI(ctrl_RI),
                     .Ov(1'b0),
                     .BP(1'b0),
                     .AdEL_inst(pc[1:0]?1'b1:1'b0),
                     .AdEL_data(1'b0),
                     .AdES(1'b0),
                     .CpU(1'b0));

   assign r_v0 = 32'h0a; // Good enough for now. To support syscall for real,
                         // you should read the syscall
                         // argument from $v0 of the register file 

   syscall_unit SU(.syscall_halt(syscall_halt), .pc(pc), .clk(clk), .Sys(ctrl_Sys),
                   .r_v0(r_v0), .rst_b(rst_b));
   assign        internal_halt = exception_halt | syscall_halt;
   register #(1, 0) Halt(halted, internal_halt, clk, 1'b1, rst_b);
   register #(32, 0) EPCReg(epc, pc, clk, load_ex_regs, rst_b);
   register #(32, 0) CauseReg(cause,
                              {25'b0, cause_code, 2'b0}, 
                              clk, load_ex_regs, rst_b);
   register #(32, 0) BadVAddrReg(bad_v_addr, pc, clk, load_bva, rst_b);

endmodule //mips_core

   

////
//// mips_ALU: Performs all arithmetic and logical operations
////
//// out (output) - Final result
//// in1 (input)  - Operand modified by the operation
//// in2 (input)  - Operand used (in arithmetic ops) to modify in1
//// sel (input)  - Selects which operation is to be performed
////
module mips_ALU(alu__out, alu__op1, alu__op2, alu__sel, alu__shiftby, alu__outbr);

   output reg [31:0] alu__out;
   output reg alu__outbr;
   input [31:0]  alu__op1, alu__op2;
   input [3:0]   alu__sel;
   input [4:0]   alu__shiftby;
   always @(*) begin
   case(alu__sel)
    4'b0000:
      //adder AdderUnit(alu__out, alu__op1, alu__op2, alu__sel[0]);
       alu__out = (alu__op1 + alu__op2);
    4'b0001:
       alu__out = (alu__op1 - alu__op2);
    4'b0010:
       alu__out = (alu__op1 & alu__op2);
    4'b0011:
       alu__out = (alu__op1 | alu__op2);
    4'b0100:
       alu__out = (!(alu__op1 | alu__op2));
    4'b0101:
       alu__out = (alu__op1 ^ alu__op2);
    4'b0110:
       alu__out = (alu__op1 < alu__op2)? 1 : 0;
    4'b0111:
       alu__out = (alu__op1 << alu__shiftby);
    4'b1000:
       alu__out = (alu__op1 >> alu__shiftby);
    4'b1001:
       alu__out = (alu__op1 >> alu__shiftby);
    4'b1010:
       alu__outbr = (alu__op1 == alu__op2) ? 1 : 0;
    4'b1011:
       alu__outbr = (alu__op1 != alu__op2) ? 1 : 0;
    4'b1100:
       alu__outbr = (alu__op1[31] == 0) ? 1 : 0;
    4'b1101:
       alu__outbr = (alu__op1 > 0) ? 1 : 0;
    4'b1110:
       alu__outbr = (alu__op1[31] == 1) ? 1 : 0;
    4'b1111:
       alu__outbr = (alu__op1 <= 0) ? 1 : 0;
    default:
       alu__out = 32'h00000000;
   endcase// endcase(alu__sel)
   end
endmodule


//// register: A register which may be reset to an arbirary value
////
//// q      (output) - Current value of register
//// d      (input)  - Next value of register
//// clk    (input)  - Clock (positive edge-sensitive)
//// enable (input)  - Load new value?
//// reset  (input)  - System reset
////
module register(q, d, clk, enable, rst_b);

   parameter
            width = 32,
            reset_value = 0;

   output [(width-1):0] q;
   reg [(width-1):0]    q;
   input [(width-1):0]  d;
   input                 clk, enable, rst_b;

   always @(posedge clk or negedge rst_b)
     if (~rst_b)
       q <= reset_value;
     else if (enable)
       q <= d;

endmodule // register


////
//// adder
////
//// out (output) - adder result
//// in1 (input)  - Operand1
//// in2 (input)  - Operand2
//// sub (input)  - Subtract?
////
module adder(out, in1, in2, sub);
   output [31:0] out;
   input [31:0]  in1, in2;
   input         sub;

   assign        out = sub?(in1 - in2):(in1 + in2);

endmodule // adder


////
//// add_const: An adder that adds a fixed constant value
////
//// out (output) - adder result
//// in  (input)  - Operand
////
module add_const(out, in);

   parameter add_value = 1;

   output   [31:0] out;
   input    [31:0] in;

   assign   out = in + add_value;

endmodule // adder



////
//// mips_mux4x1
////
module mips_mux4x1(out,sel,in1,in2,in3);
   output reg [31:0] out;
   input [31:0] in1, in2, in3;
   input [1:0] sel;
   always @(*) begin
   case(sel)
      2'b00:
	 out = in1;
      2'b01:
	 out = in2;
      2'b10:
	 out = in3;
      default: 
	 out = in1;
   endcase // case(sel)
   end
endmodule // mux4x1



// Local Variables:
// verilog-library-directories:("." "../447rtl")
// End:
