
//////////////////////////////////////////////////////////////////////
////                                                              ////
//// registerInterface.v                                          ////
////                                                              ////
//// This file is part of the i2cSlave opencores effort.
////                                                              ////
//// Module Description:                                          ////
//// You will need to modify this file to implement your
//// interface.
//// Add your control and status bytes/bits to module inputs and outputs,
//// and also to the I2C read and write process blocks
////                                                              ////
//// To Do:                                                       ////
////
////                                                              ////
//// Author(s):                                                   ////
//// - Steve Fielding, sfielding@base2designs.com                 ////
////                                                              ////
//////////////////////////////////////////////////////////////////////
////                                                              ////
////                                                              ////
//// This source file may be used and distributed without         ////
//// restriction provided that this copyright statement is not    ////
//// removed from the file and that any derivative work contains  ////
//// the original copyright notice and the associated disclaimer. ////
////                                                              ////
//// This source file is free software; you can redistribute it   ////
//// and/or modify it under the terms of the GNU Lesser General   ////
//// Public License as published by the Free Software Foundation; ////
//// either version 2.1 of the License, or (at your option) any   ////
//// later version.                                               ////
////                                                              ////
//// This source is distributed in the hope that it will be       ////
//// useful, but WITHOUT ANY WARRANTY; without even the implied   ////
//// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      ////
//// PURPOSE. See the GNU Lesser General Public License for more  ////
//// details.                                                     ////
////                                                              ////                                                            
//////////////////////////////////////////////////////////////////////
//
`include "i2cSlave_define.v"

module registerInterface (
  rst,
  clk,
  addr,
  dataIn,
  writeEn,
  dataOut,
  myReg0,
  myReg1,
  myReg2,
  myReg3,
  myReg4,
  myReg5,
  myReg6,
  videoUpdateToggle
);
input rst;
input clk;
input [7:0] addr;
input [7:0] dataIn;
input writeEn;
output [7:0] dataOut;
output [7:0] myReg0;
output [7:0] myReg1;
input [7:0] myReg2;
output [7:0] myReg3;
output [7:0] myReg4;
output [7:0] myReg5;
output [7:0] myReg6;
output videoUpdateToggle;

reg [7:0] dataOut;
reg [7:0] myReg0;
reg [7:0] myReg1;
reg [7:0] myReg3;
reg [7:0] myReg4;
reg [7:0] myReg5;
reg [7:0] myReg6;
reg videoUpdateToggle;

// --- I2C Read
always @(posedge clk) begin
  case (addr)
    8'h00: dataOut <= myReg0;
    8'h01: dataOut <= myReg1;
    8'h02: dataOut <= myReg2;  //only read
    8'h03: dataOut <= myReg3;  // bit[0]: preprocessing master enable
    8'h04: dataOut <= myReg4;  // bit[0]: plate box drawing enable
    8'h05: dataOut <= myReg5;  // bit[0]: white balance feature enable
    8'h06: dataOut <= myReg6;  // bit[0]: dark-channel dehaze feature enable
    default: dataOut <= 8'h00;
  endcase
end

// --- I2C Write
always @(posedge clk or posedge rst) begin
 if(rst)
   begin
    	myReg0<=0;
		myReg1<=0;
    myReg3<=0;
    myReg4<=0;
    myReg5<=0;
    myReg6<=0;
    videoUpdateToggle<=1'b0;
  end
else
  if (writeEn == 1'b1) begin
    case (addr)
      8'h00: myReg0 <= dataIn;
      // 0x01[0]: 0 = camera, 1 = HDMI. Toggle on every write so a
      // repeated ARM command still overrides a previous key selection.
      8'h01: begin
        myReg1 <= dataIn;
        videoUpdateToggle <= ~videoUpdateToggle;
      end
      8'h03: myReg3 <= dataIn; // bit[0]: preprocessing master
      8'h04: myReg4 <= dataIn; // bit[0]: plate box drawing master
      // These feature bits take effect only while 0x03[0] is enabled.
      8'h05: myReg5 <= dataIn; // bit[0]: white balance
      8'h06: myReg6 <= dataIn; // bit[0]: dark-channel dehaze
    endcase
  end
end

endmodule

