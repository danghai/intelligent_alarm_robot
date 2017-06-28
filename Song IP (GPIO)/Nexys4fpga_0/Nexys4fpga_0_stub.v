// Copyright 1986-2016 Xilinx, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2016.2 (win64) Build 1577090 Thu Jun  2 16:32:40 MDT 2016
// Date        : Tue Jun 06 21:05:41 2017
// Host        : danghai running 64-bit major release  (build 9200)
// Command     : write_verilog -force -mode synth_stub
//               c:/Users/haidang/Desktop/project2/colorwheel/colorwheel.srcs/sources_1/ip/Nexys4fpga_0/Nexys4fpga_0_stub.v
// Design      : Nexys4fpga_0
// Purpose     : Stub declaration of top-level module interface
// Device      : xc7a100tcsg324-1
// --------------------------------------------------------------------------------

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
(* X_CORE_INFO = "Nexys4fpga,Vivado 2016.2" *)
module Nexys4fpga_0(clk, sysreset, playlist_no_wire, alarm_beep_wire, alarm_en_wire, aud_en_wire, AUD_PWM, AUD_SD)
/* synthesis syn_black_box black_box_pad_pin="clk,sysreset,playlist_no_wire[19:0],alarm_beep_wire,alarm_en_wire,aud_en_wire,AUD_PWM,AUD_SD" */;
  input clk;
  input sysreset;
  input [19:0]playlist_no_wire;
  input alarm_beep_wire;
  input alarm_en_wire;
  input aud_en_wire;
  output AUD_PWM;
  output AUD_SD;
endmodule
